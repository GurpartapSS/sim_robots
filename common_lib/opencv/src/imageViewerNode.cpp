#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageViewerNode : public rclcpp::Node
{
public:
    ImageViewerNode() : Node("image_viewer_node"),test(true)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(2)),
            std::bind(&ImageViewerNode::imageCallback, this, std::placeholders::_1));

        cv::namedWindow("Camera Image");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if(!cv_ptr->image.empty()) {
                cv::imshow("Camera Image", cv_ptr->image);
                cv::waitKey(1);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unhandled exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    bool test;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
