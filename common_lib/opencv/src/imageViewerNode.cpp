#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageViewerNode : public rclcpp::Node
{
public:
    ImageViewerNode() : Node("image_viewer_node"),test(true)
    {
        this->declare_parameter("depth",0);
        depth = this->get_parameter("depth").as_int();

        std::string topic_ = depth ? "/camera/depth/image_raw" : "/camera/image_raw";
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_,
            rclcpp::QoS(rclcpp::KeepLast(2)),
            std::bind(&ImageViewerNode::imageCallback, this, std::placeholders::_1));

        cv::namedWindow("Camera Image");
        RCLCPP_INFO(this->get_logger(),"Image Viewer started from topic %s",topic_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            if (depth) {
                cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                cv::normalize(cv_ptr_->image, depth_image_normalized_, 0, 1, cv::NORM_MINMAX);
            } else {
                cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            if(!cv_ptr_->image.empty()) {
                if(depth) {
                    cv::imshow("Camera Image", depth_image_normalized_);
                } else {
                    cv::imshow("Camera Image", cv_ptr_->image);
                }
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
    int depth;
    cv::Mat depth_image_normalized_;
    cv_bridge::CvImagePtr cv_ptr_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
