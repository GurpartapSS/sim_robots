#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roboarm_remote/action/arm_joints.hpp"
#include "inverse_kine/msg/cartisan_coord.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

#define M_PI 3.14159265358979323846


using ArmJoints = roboarm_remote::action::ArmJoints;
using GoalHandle = rclcpp_action::ClientGoalHandle<ArmJoints>;
using namespace std::placeholders;

class ik
{
public:
    ik(int j1, int j2) : j1_(j1), j2_(j2), orientation_(0)
    {
    }
    // Z up X Y on ground
    // return: m1 - b_static in XY, m2 - a1 in YZ, m3 - a2 in YZ, m4 - orientation in in YZ
    std::vector<float> convert(double x, double y, double d)
    {
        std::vector<float> port;
        port.push_back(atan2(d, y));                      // - m1
        port.push_back(M_PI / 2 - getAngle(j1_, d, j2_)); // - m2
        port.push_back(M_PI - getAngle(j1_, j2_, d));     // - m3
        port.push_back(getAngle(j2_, d, j1_)-(M_PI/2));            // - m4

        return port;
    }

private:
    int j1_;          // length1
    int j2_;          // length2
    int orientation_; // position wrt obj, 0 is standing
    static const int hand_ca = 0;

    float getAngle(double a, double b, double op)
    {
        return acos(((a * a) + (b * b) - (op * op)) / (2 * a * b));
    }
};

class CartisanToJointsNode : public rclcpp::Node
{
public:
    CartisanToJointsNode() : Node("cartisan_to_joints"), converter_(25, 25)
    {
        count_until_client_ = rclcpp_action::create_client<ArmJoints>(this,
        "arm_joints");
        subscriber_ = this->create_subscription<inverse_kine::msg::CartisanCoord>("cart_coords", 10,
        std::bind(&CartisanToJointsNode::conversionCallback,this, _1));
    }
    void send_goal(std::vector<float> joints) {
        //wati for server
        count_until_client_->wait_for_action_server();

        //add goal
        auto goal = ArmJoints::Goal();
        joints.push_back(0.0);
        joints.push_back(0.016);
        RCLCPP_INFO(this->get_logger(),"Joints %f %f %f %f %f %f",joints[0],joints[1],joints[2],
        joints[3],joints[4],joints[5]);
        goal.joints = joints;

        //add goal options
        auto goal_options = rclcpp_action::Client<ArmJoints>::SendGoalOptions();
        goal_options.result_callback = std::bind(&CartisanToJointsNode::goalCallBack, this, _1);
        goal_options.goal_response_callback = std::bind(&CartisanToJointsNode::goalResponseCallBack, this, _1);
        goal_options.feedback_callback = std::bind(&CartisanToJointsNode::feedbackCallBack, this, _1, _2);

        RCLCPP_INFO(this->get_logger(),"Sending a goal");
        auto future_goal_handle = count_until_client_->async_send_goal(goal, goal_options);
        // timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CartisanToJointsNode::timercallback, this));

    }

private:
    rclcpp_action::Client<ArmJoints>::SharedPtr count_until_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    GoalHandle::SharedPtr goal_;
    std::vector<int> goals_;
    ik converter_;
    rclcpp::Subscription<inverse_kine::msg::CartisanCoord>::SharedPtr subscriber_;
    void conversionCallback(const inverse_kine::msg::CartisanCoord::SharedPtr coords) {
        RCLCPP_INFO(this->get_logger(),"Got the coordinates: %f %f %f",coords->x, coords->y, coords->z);
        std::vector<float> joints_ = converter_.convert(coords->x,coords->y,coords->z);
        send_goal(joints_);
    };
    void goalCallBack(const GoalHandle::WrappedResult &result) {

        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),"Goal Succeeded");
        } else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(),"Goal Aborted");
        } else if(status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(),"Goal Canceled");
        }
            int reached_number = result.result->success;
    }

    void goalResponseCallBack(const GoalHandle::SharedPtr &goal_handle) {
        if(!goal_handle) {
            RCLCPP_INFO(this->get_logger(),"Goal Rejected :(");
        } else {
            RCLCPP_INFO(this->get_logger(),"Goal Accepted :D");
            goal_ = goal_handle;
        }
    }

    void feedbackCallBack( const GoalHandle::SharedPtr &goal_handle,
    const std::shared_ptr<const ArmJoints::Feedback> feedback) {
        (void)goal_handle;
        // int number = feedback->precentage;
        // RCLCPP_INFO(this->get_logger(),"Feedback %d",number);
    }

    void timercallback() {
        // RCLCPP_INFO(this->get_logger(),"cancel goal");
        // count_until_client_->async_cancel_goal(goal_);
        timer_->cancel();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartisanToJointsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
