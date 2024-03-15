#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roboarm_remote/action/arm_task.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using ArmTask = roboarm_remote::action::ArmTask;
using ArmTaskGoalHandle = rclcpp_action::ServerGoalHandle<ArmTask>;
using namespace std::placeholders;

class ArmTaskServerNode : public rclcpp::Node
{
public:
    ArmTaskServerNode() : Node("arm_task_server")
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        count_until_server_ = rclcpp_action::create_server<ArmTask>(
            this,
            "arm_task",
            std::bind(&ArmTaskServerNode::goalCallback, this, _1, _2),
            std::bind(&ArmTaskServerNode::cancelCallBack, this, _1),
            std::bind(&ArmTaskServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(), cb_group_);
            RCLCPP_INFO(this->get_logger(),"Action Server Started");
    }

private:

    rclcpp_action::Server<ArmTask>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ArmTask::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            RCLCPP_INFO(this->get_logger(),"Received a goal");

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse cancelCallBack(
        const std::shared_ptr<ArmTaskGoalHandle> goal_handle)
        {
            (void) goal_handle;
            RCLCPP_INFO(this->get_logger(),"Cancel Request accepted");

            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"arm");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"gripper");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted_callback(const std::shared_ptr<ArmTaskGoalHandle> goal_handle)
    {
        execute_goal(goal_handle);
    }
    void execute_goal(const std::shared_ptr<ArmTaskGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(),"Executing GOal");
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"arm");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(),"gripper");

        std::vector<double> arm_joint_goal;
        std::vector<double> gripper_joint_goal;

        if(goal_handle->get_goal()->task_number == 0) {
            arm_joint_goal = {0.8, 0.36, 1.57, -0.36, 0.0};
            gripper_joint_goal = {0.15, -0.15};
        } else if (goal_handle->get_goal()->task_number == 1)
        {
            arm_joint_goal = {0.8, 0.36, 1.57, -0.36, 0.0};
            gripper_joint_goal = {0.05, -0.05};
        } else if (goal_handle->get_goal()->task_number == 2)
        {
            arm_joint_goal = {0.0, 0.0, 0.0, 1.57, 0.0};
            gripper_joint_goal = {0.05, -0.05};
        } else {
            RCLCPP_ERROR(this->get_logger(),"Wrong goal state requested");
            return;
        }

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

        auto result = std::make_shared<ArmTask::Result>();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),"Goal Succeded!!");

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTaskServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}