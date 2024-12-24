#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <memory>
#include <iostream>
#include <sstream>

class FibonacciActionClient : public rclcpp::Node {
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    FibonacciActionClient() : Node("fibonacci_action_client") {
        client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

        // Ждем, пока action-сервер будет доступен
        while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to become available...");
        }

        RCLCPP_INFO(this->get_logger(), "Action server is now available. Please enter the sequence length:");

        // Читаем ввод пользователя
        int sequence_length;
        std::cin >> sequence_length;

        // Отправляем запрос
        send_goal(sequence_length);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;

    void send_goal(int sequence_length) {
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = sequence_length;

        RCLCPP_INFO(this->get_logger(), "Sending goal to calculate Fibonacci sequence of length %d", sequence_length);

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleFibonacci::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server.");
        }
    }

    void feedback_callback(GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        // Используем вспомогательную функцию для преобразования sequence в строку
        std::string feedback_str = vector_to_string(feedback->sequence);
        RCLCPP_INFO(this->get_logger(), "Received feedback: current sequence is [%s]", feedback_str.c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Action completed successfully.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Action was aborted by the server.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Action was canceled by the server.");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }

        // Преобразуем результат в строку
        std::string result_str = vector_to_string(result.result->sequence);
        RCLCPP_INFO(this->get_logger(), "Final Fibonacci sequence: [%s]", result_str.c_str());
    }

    // Вспомогательная функция для преобразования std::vector<int> в строку
    std::string vector_to_string(const std::vector<int>& vec) {
        std::stringstream ss;
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << vec[i];
        }
        return ss.str();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FibonacciActionClient>());
    rclcpp::shutdown();
    return 0;
}
