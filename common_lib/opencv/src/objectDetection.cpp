#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/dnn.hpp"
#include <fstream>

using namespace cv;
using namespace dnn;

class ImageViewerNode : public rclcpp::Node
{
public:
    ImageViewerNode() : Node("object_detection_node"), test(true)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(2)),
            std::bind(&ImageViewerNode::imageCallback, this, std::placeholders::_1));

        net = cv::dnn::readNetFromDarknet(modelConfig, modelWeights);

        std::ifstream ifs(classesFile.c_str());
        while (getline(ifs, line))
            classes.push_back(line);

        cv::namedWindow("Camera Image");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    bool test;
    const std::string modelWeights = "/home/igris/maya_ws/src/opencv/yolo_config/yolov3.weights";
    const std::string modelConfig = "/home/igris/maya_ws/src/opencv/yolo_config/yolov3.cfg";
    const std::string classesFile = "/home/igris/maya_ws/src/opencv/yolo_config/coco.names";
    Net net;

    std::vector<std::string> classes;
    std::string line;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if (!cv_ptr->image.empty())
            {
                cv::Mat frame;
                cv_ptr->image;
                Mat blob;
                blobFromImage(cv_ptr->image, blob, 1 / 255.0, Size(320, 320), Scalar(0, 0, 0), true, false);

                net.setInput(blob);

                std::vector<Mat> outs;
                net.forward(outs, getOutputsNames(net));

                postprocess(cv_ptr->image, outs, classes, 0.8); // Confidence threshold set to 80%

                imshow("Camera Image", cv_ptr->image);
                cv::waitKey(1);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unhandled exception: %s", e.what());
        }
    }


    void postprocess(Mat &frame, const std::vector<Mat> &outs, const std::vector<std::string> &classes, float confThreshold = 0.5, float nmsThreshold = 0.4)
    {
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<Rect> boxes;
        for (size_t i = 0; i < outs.size(); ++i)
        {
            float *data = (float *)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > confThreshold)
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }
        std::vector<int> indices;
        NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            Rect box = boxes[idx];
            int classId = classIds[idx];
            float confidence = confidences[idx];
            // Draw bounding box and label
            rectangle(frame, box, Scalar(0, 255, 0), 2);
            String label = format("%s: %.2f", classes[classId].c_str(), confidence);
            int baseLine;
            Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            int top = max(box.y, labelSize.height);
            putText(frame, label, Point(box.x, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }

    std::vector<std::string> getOutputsNames(const Net &net)
    {
        static std::vector<std::string> names;
        if (names.empty())
        {
            std::vector<int> outLayers = net.getUnconnectedOutLayers();
            std::vector<String> layersNames = net.getLayerNames();
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
