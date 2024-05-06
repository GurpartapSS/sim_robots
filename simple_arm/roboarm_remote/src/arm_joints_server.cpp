#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roboarm_remote/action/arm_joints.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using ArmJoints = roboarm_remote::action::ArmJoints;
using ArmJointsGoalHandle = rclcpp_action::ServerGoalHandle<ArmJoints>;
using namespace std::placeholders;

class ArmJointsServerNode : public rclcpp::Node
{
public:
    ArmJointsServerNode() : Node("arm_joints_server")
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        count_until_server_ = rclcpp_action::create_server<ArmJoints>(
            this,
            "arm_joints",
            std::bind(&ArmJointsServerNode::goalCallback, this, _1, _2),
            std::bind(&ArmJointsServerNode::cancelCallBack, this, _1),
            std::bind(&ArmJointsServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(), cb_group_);
            RCLCPP_INFO(this->get_logger(),"Action Server Started");
    }

private:

    rclcpp_action::Server<ArmJoints>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ArmJoints::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            RCLCPP_INFO(this->get_logger(),"Received a goal");

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse cancelCallBack(
        const std::shared_ptr<ArmJointsGoalHandle> goal_handle)
        {
            (void) goal_handle;
            RCLCPP_INFO(this->get_logger(),"Cancel Request accepted");

            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"arm");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"gripper");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted_callback(const std::shared_ptr<ArmJointsGoalHandle> goal_handle)
    {
        execute_goal(goal_handle);
    }
    void execute_goal(const std::shared_ptr<ArmJointsGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(),"Executing GOal");
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"arm");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"gripper");

        std::vector<double> arm_joint_goal;
        std::vector<double> gripper_joint_goal;

        // Todo: check for the limits on joints

            arm_joint_goal = {goal_handle->get_goal()->joints[0], goal_handle->get_goal()->joints[1],
             goal_handle->get_goal()->joints[2], -(goal_handle->get_goal()->joints[3]), goal_handle->get_goal()->joints[4]};
            gripper_joint_goal = {goal_handle->get_goal()->joints[5], -(goal_handle->get_goal()->joints[5])};

        if(!arm_move_group.setJointValueTarget(arm_joint_goal) ||
        !gripper_move_group.setJointValueTarget(gripper_joint_goal)) {
            RCLCPP_WARN(this->get_logger(),"target joint position not reachable");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

        if( !(arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS ) ||
        !(gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)) {
            RCLCPP_WARN(this->get_logger(),"one or more plan failed");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(),"Executing task!!");
            gripper_move_group.move();
            arm_move_group.move();

        }

        auto result = std::make_shared<ArmJoints::Result>();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),"Goal Succeded!!");

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmJointsServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}