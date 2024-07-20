#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"

class simpleController : public rclcpp::Node {
    public:
        simpleController() : Node("simple_controller"), wheel_radius_(0.0325), wheel_separation_(.13)
        {
            wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands",10);
            vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("nemo_controller/cmd_vel",10,
                std::bind(&simpleController::velCallback, this, std::placeholders::_1));
            speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;

            RCLCPP_INFO_STREAM(this->get_logger(),"The conversion matrix is \n" << speed_conversion_);
        }
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

        double wheel_radius_;
        double wheel_separation_;
        Eigen::Matrix2d speed_conversion_;

        void velCallback(const  geometry_msgs::msg::TwistStamped & msg) {
            Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

            Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

            std_msgs::msg::Float64MultiArray wheel_speed_msg;
            wheel_speed_msg.data.push_back(wheel_speed.coeff(1)); //veloity of left wheel
            wheel_speed_msg.data.push_back(wheel_speed.coeff(0)); //velocity of right wheel

            wheel_cmd_pub_->publish(wheel_speed_msg);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simpleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}