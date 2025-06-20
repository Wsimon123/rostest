#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_action/action/fibonacci.hpp"

using Fibonacci = my_action::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ActionServer : public rclcpp::Node {
public:
  ActionServer() : Node("fibonacci_action_server") {
    // 创建 Action Server
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  // 处理新目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Fibonacci::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 执行 Action
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    std::thread{std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Fibonacci::Result>();
    auto feedback = std::make_shared<Fibonacci::Feedback>();

    feedback->partial_sequence = {0, 1};
    for (int i = 1; i < goal->order; ++i) {
      if (goal_handle->is_canceling()) {
        result->sequence = feedback->partial_sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->partial_sequence.push_back(
        feedback->partial_sequence[i] + feedback->partial_sequence[i - 1]);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    result->sequence = feedback->partial_sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}