#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
using namespace std::chrono;

using lcCallback = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class SimpleifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit SimpleifecycleNode(const std::string & name, bool intra_process_com = false) : 
    rclcpp_lifecycle::LifecycleNode(name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_com)) {

    }

    lcCallback on_configure(const rclcpp_lifecycle::State &) {
        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10,
        std::bind(&SimpleifecycleNode::msgscallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node ONCONFIGURE called");
        return lcCallback::SUCCESS;
    }
    
    lcCallback on_shutdown(const rclcpp_lifecycle::State &) {
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node ONSHUTDOWN called");
        return lcCallback::SUCCESS;
    }

    lcCallback on_cleanup(const rclcpp_lifecycle::State &) {
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node ONCLEANUP called");
        return lcCallback::SUCCESS;
    }

    lcCallback on_activate(const rclcpp_lifecycle::State &state) {
        LifecycleNode::on_activate(state);
        std::this_thread::sleep_for(2s);
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node ACTIVATED!");
        return lcCallback::SUCCESS;
    }

    lcCallback on_deactivate(const rclcpp_lifecycle::State &state) {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node DEACTIVATED");
        return lcCallback::SUCCESS;
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void msgscallback(const std_msgs::msg::String &msg) {
        auto state = this->get_current_state();
        if(state.label() == "active") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Lifecycle Node heard " << msg.data.c_str());
        }
    }
};

int main(int argc, char* argv[]) {
    
    rclcpp::init(argc,argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<SimpleifecycleNode> simple_lifecycle_node = std::make_shared<SimpleifecycleNode>("simple_LC_node");
    ste.add_node(simple_lifecycle_node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0;
}