#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

class FibonacciActionServer : public rclcpp::Node {
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    FibonacciActionServer() : Node("fibonacci_action_server") {
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Action server 'fibonacci_action' started.");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal
    ) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle
    ) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        std::thread([this, goal_handle]() {
            auto feedback = std::make_shared<Fibonacci::Feedback>();
            auto &sequence = feedback->sequence;

            sequence.push_back(0);
            sequence.push_back(1);

            for (int i = 2; i < goal_handle->get_goal()->order; ++i) {
                sequence.push_back(sequence[i - 1] + sequence[i - 2]);
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            auto result = std::make_shared<Fibonacci::Result>();
            result->sequence = sequence;

            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
        }).detach();
    }

    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FibonacciActionServer>());
    rclcpp::shutdown();
    return 0;
}
