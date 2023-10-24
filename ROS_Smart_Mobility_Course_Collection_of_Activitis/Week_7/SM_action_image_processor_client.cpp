#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/image_processing.hpp>

class ImageProcessorClient : public rclcpp::Node
{
public:
    explicit ImageProcessorClient()
        : Node("image_processor_client")
    {
        action_client_ = rclcpp_action::create_client<example_interfaces::action::ImageProcessing>(
            this, "image_processing");

        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(get_logger(), "Waiting for action server to start...");
        }
    }

    void send_goal()
    {
        auto goal_msg = example_interfaces::action::ImageProcessing::Goal();
        auto goal_handle_future = action_client_->async_send_goal(goal_msg);

        if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to send goal.");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Goal accepted by server. Waiting for result...");
        auto result_future = action_client_->async_get_result(goal_handle);

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get result.");
            return;
        }

        auto result = result_future.get();
        RCLCPP_INFO(get_logger(), "Received result from server.");
    }

private:
    rclcpp_action::Client<example_interfaces::action::ImageProcessing>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto image_processor_client = std::make_shared<ImageProcessorClient>();
    image_processor_client->send_goal();
    rclcpp::shutdown();
    return 0;
}

