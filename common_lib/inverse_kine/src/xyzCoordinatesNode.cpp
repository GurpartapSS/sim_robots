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
    // h - height, d - depth, y - position on floor
    // return: m1 - b_static in XY, m2 - a1 in YZ, m3 - a2 in YZ, m4 - orientation in in YZ, please follow the reference image
    std::vector<float> convert(double h, double y, double d)
    {
        std::vector<float> port;
        float raise_base = sqrt(d*d + h*h);
        float m1_adj = atan2(h, d);                                                  // angle by which the triangle is lifted
        port.push_back(atan2(y, d));                                                 // m1 - movement of base along the floor left/right
        port.push_back(M_PI / 2 - getAngle(j1_, raise_base, j2_) - m1_adj);          // m2 - angle to move from 90deg rest position, front/back 
        port.push_back(M_PI - getAngle(j1_, j2_, raise_base));                       // m3 - angle to move wrt arm1 rest alignment, front/back
        float wrist_adj = getAngle(j2_, raise_base, j1_) - (M_PI/2 - atan2(d,h)); 
        port.push_back(wrist_adj);                                      // m4 - angle to move wrt arm2 keeping horizzontal alignment, up/down

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
    CartisanToJointsNode() : Node("cartisan_to_joints"), converter_(12, 12)
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
        (void)feedback;
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
