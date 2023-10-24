#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/image_processing.hpp>

class ImageProcessorServer : public rclcpp::Node
{
public:
    explicit ImageProcessorServer()
        : Node("image_processor_server")
    {
        action_server_ = create_server<example_interfaces::action::ImageProcessing>(
            "image_processing",
            std::bind(&ImageProcessorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ImageProcessorServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ImageProcessorServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Server<example_interfaces::action::ImageProcessing>::SharedPtr action_server_;

    void handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const example_interfaces::action::ImageProcessing::Goal> goal)
    {
        // Process the image (edge detection)
        // Implement image processing logic here

        // Publish result
        auto result = std::make_shared<example_interfaces::action::ImageProcessing::Result>();
        action_server_->succeeded(goal, result);
    }

    void handle_cancel(const rclcpp_action::GoalUUID &)
    {
        RCLCPP_INFO(get_logger(), "Goal was canceled.");
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::ImageProcessing>> goal_handle)
    {
        // Goal accepted callback
        RCLCPP_INFO(get_logger(), "Goal accepted.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessorServer>());
    rclcpp::shutdown();
    return 0;
}

