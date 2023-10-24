#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/image_processing.hpp>

class ImageProcessingServerNode : public rclcpp::Node
{
public:
    explicit ImageProcessingServerNode()
        : Node("image_processing_server_node")
    {
        image_processing_action_server_ = create_server<example_interfaces::action::ImageProcessing>(
            "image_processing_action",
            std::bind(&ImageProcessingServerNode::process_image_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ImageProcessingServerNode::cancel_image_processing, this, std::placeholders::_1),
            std::bind(&ImageProcessingServerNode::handle_accepted_goal, this, std::placeholders::_1));
    }

private:
    rclcpp::Server<example_interfaces::action::ImageProcessing>::SharedPtr image_processing_action_server_;

    void process_image_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const example_interfaces::action::ImageProcessing::Goal> image_goal)
    {
        // Implement image processing logic here
        // Process the image (e.g., edge detection)

        // Publish result
        auto image_result = std::make_shared<example_interfaces::action::ImageProcessing::Result>();
        image_processing_action_server_->succeeded(image_goal, image_result);

        RCLCPP_INFO(get_logger(), "Image processing goal succeeded.");
    }

    void cancel_image_processing(const rclcpp_action::GoalUUID &)
    {
        RCLCPP_INFO(get_logger(), "Image processing goal was canceled.");
    }

    void handle_accepted_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::ImageProcessing>> goal_handle)
    {
        // Goal accepted callback
        RCLCPP_INFO(get_logger(), "Image processing goal accepted.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessingServerNode>());
    rclcpp::shutdown();
    return 0;
}

