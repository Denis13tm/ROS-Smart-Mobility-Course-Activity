#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/image_processing.hpp>

class ImageProcessorNode : public rclcpp::Node
{
public:
    explicit ImageProcessorNode()
        : Node("image_processor_node")
    {
        image_action_client_ = rclcpp_action::create_client<example_interfaces::action::ImageProcessing>(
            this, "image_processing_action");

        while (!image_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(get_logger(), "Waiting for action server to initialize...");
        }
    }

    void processAndSendImage()
    {
        auto image_goal = example_interfaces::action::ImageProcessing::Goal();
        auto image_goal_future = image_action_client_->async_send_goal(image_goal);

        if (rclcpp::spin_until_future_complete(shared_from_this(), image_goal_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to send image processing goal.");
            return;
        }

        auto goal_handle = image_goal_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Image processing goal accepted by server. Waiting for result...");
        auto result_future = image_action_client_->async_get_result(goal_handle);

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get image processing result.");
            return;
        }

        auto result = result_future.get();
        RCLCPP_INFO(get_logger(), "Received image processing result from server.");
    }

private:
    rclcpp_action::Client<example_interfaces::action::ImageProcessing>::SharedPtr image_action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto image_processor_node = std::make_shared<ImageProcessorNode>();
    image_processor_node->processAndSendImage();
    rclcpp::shutdown();
    return 0;
}


