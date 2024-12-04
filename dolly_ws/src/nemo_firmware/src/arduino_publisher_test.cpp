#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "libserial/SerialPort.h"


class simpleArduinoPubNode : public rclcpp::Node
{
public:
    simpleArduinoPubNode() : Node("simpleArduinoPubNode")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = this->get_parameter("port").as_string();

        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        sub_ = this->create_subscription<std_msgs::msg::String>("simple_serial_transmitter", 10, 
        std::bind(&simpleArduinoPubNode::subCallback, this, std::placeholders::_1));
    }

    ~simpleArduinoPubNode() {
        arduino_.Close();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    LibSerial::SerialPort arduino_;
    std::string port_;
    
    void subCallback(const std_msgs::msg::String &msg) {
        arduino_.Write(msg.data);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simpleArduinoPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}