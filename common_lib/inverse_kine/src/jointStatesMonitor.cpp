#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateMonitor : public rclcpp::Node
{
public:
    JointStateMonitor() : Node("sub_JointStates"), start_time(std::chrono::steady_clock::now())
    {
        jointStatesSub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
        std::bind(&JointStateMonitor::jointStatesCallback,this,std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),"Subscribed to Joint States!");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSub_;
    std::shared_ptr<sensor_msgs::msg::JointState> cachedJointStates_;
    std::chrono::steady_clock::time_point start_time;
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr jointStates) {
        static int first = 0;
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() <= 1000) { 
            return;
        }
        start_time = current_time;
        if(first == 0){
            cachedJointStates_ = jointStates;
            first = 1;
        }
        if(std::equal(cachedJointStates_->position.begin(),cachedJointStates_->position.end(),
        jointStates->position.begin())) { 
            return;
        }
        // movement logic here
        RCLCPP_INFO(this->get_logger(),"Movement %lf", cachedJointStates_->position[2]);
        cachedJointStates_ = jointStates;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}