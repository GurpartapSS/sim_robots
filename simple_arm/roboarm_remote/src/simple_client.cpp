#include"rclcpp/rclcpp.hpp"
#include"rclcpp_action/rclcpp_action.hpp"
#include"roboarm_remote/action/arm_task.hpp"

using ArmTask = roboarm_remote::action::ArmTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<ArmTask>;
using namespace std::placeholders;

class CountUnitlClient : public rclcpp::Node
{
public:
    CountUnitlClient() : Node("sequence_task")
    {
        count_until_client_ = rclcpp_action::create_client<ArmTask>(this,
        "arm_task");
    }
    void set_goals(std::vector<int> goals) {
        goals_ = goals;
        int goal = goals_.back();
        goals_.pop_back();
        send_goal(goal);
    }
    void send_goal(int target_number) {
        //wati for server
        count_until_client_->wait_for_action_server();

        //add goal
        auto goal = ArmTask::Goal();
        goal.task_number = target_number;

        //add goal options
        auto goal_options = rclcpp_action::Client<ArmTask>::SendGoalOptions();
        goal_options.result_callback = std::bind(&CountUnitlClient::goalCallBack, this, _1);
        goal_options.goal_response_callback = std::bind(&CountUnitlClient::goalResponseCallBack, this, _1);
        goal_options.feedback_callback = std::bind(&CountUnitlClient::feedbackCallBack, this, _1, _2);

        RCLCPP_INFO(this->get_logger(),"Sending a goal");
        auto future_goal_handle = count_until_client_->async_send_goal(goal, goal_options);
        // timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CountUnitlClient::timercallback, this));

    }

private:
    rclcpp_action::Client<ArmTask>::SharedPtr count_until_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    GoalHandle::SharedPtr goal_;
    std::vector<int> goals_;
    void goalCallBack(const GoalHandle::WrappedResult &result) {

        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),"Goal Succeeded");
            if(goals_.size()!=0){
                set_goals(goals_);
            }
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
    const std::shared_ptr<const ArmTask::Feedback> feedback) {
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
    auto node = std::make_shared<CountUnitlClient>();
    std::vector<int> goals = {2,1,0};
    node->set_goals(goals);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}