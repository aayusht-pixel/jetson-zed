#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

class ZedNode: public rclcpp::Node
{
    public:
    ZedNode() : Node("zed_node")
    {
        RCLCPP_INFO(this->get_logger(), "ZedNode Initialised");
    }

    private:
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedNode>();

    // Print message every second
    rclcpp::Rate rate(1.0); // 1 Hz
    while(rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "ZedNode is running ...");
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}