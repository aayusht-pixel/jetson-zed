#include "rclcpp/rclcpp.hpp"
#include <sl/Camera.hpp>

using namespace sl;

class ZedNode : public rclcpp::Node
{
public:
    ZedNode() : Node("zed_node")
    {
        // Initialize ZED camera and print serial number
        initializeZed();
    }

private:
    void initializeZed()
    {
        Camera zed;

        // Open the camera
        ERROR_CODE returned_state = zed.open();
        if (returned_state != ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Error %d: Cannot open ZED camera.", returned_state);
            return;
        }

        // Get camera information (ZED serial number)
        auto camera_infos = zed.getCameraInformation();
        RCLCPP_INFO(this->get_logger(), "Hello! This is my serial number: %d", camera_infos.serial_number);

        // Close the camera
        zed.close();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedNode>();

    // Keep the node running
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
