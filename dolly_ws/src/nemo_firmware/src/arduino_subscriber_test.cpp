#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "libserial/SerialPort.h"


class simpleArduinoSubNode : public rclcpp::Node
{
public:
    simpleArduinoSubNode() : Node("simpleArduinoSubNode"), counter_(0)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = this->get_parameter("port").as_string();

        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        pub_ = this->create_publisher<std_msgs::msg::String>("serial_msg", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&simpleArduinoSubNode::timerCallback, this));

    }

    ~simpleArduinoSubNode() {
        arduino_.Close();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int counter_;
    LibSerial::SerialPort arduino_;
    std::string port_;
    void timerCallback() {
        auto message = std_msgs::msg::String();
        if(rclcpp::ok && arduino_.IsDataAvailable()) {
            arduino_.ReadLine(message.data);
        }
        pub_->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simpleArduinoSubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}