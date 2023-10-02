#include "rclcpp/rclcpp.hpp"
#include <sl/Camera.hpp>
#include "geometry_msgs/msg/twist.hpp" // Include for Twist messages

using namespace sl;

class ZedNode : public rclcpp::Node
{
public:
    ZedNode() : Node("zed_node")
    {
        // Initialize a publisher for Twist messages
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize ZED camera and react based on detected obstacles
        detectObstacle();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_; // Twist publisher

    void detectObstacle()
    {
        Camera zed;

        // Set configuration parameters
        InitParameters init_parameters;
        init_parameters.depth_mode = DEPTH_MODE::ULTRA;
        init_parameters.coordinate_units = UNIT::MILLIMETER;

        auto returned_state = zed.open(init_parameters);
        if (returned_state != ERROR_CODE::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %d: Cannot open ZED camera.", returned_state);
            return;
        }

        sl::Mat image, depth, point_cloud;
        int i = 0;
        while (i < 50 && rclcpp::ok())
        {
            if (zed.grab() == ERROR_CODE::SUCCESS)
            {
                zed.retrieveImage(image, VIEW::LEFT);
                zed.retrieveMeasure(depth, MEASURE::DEPTH);
                zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

                int x = image.getWidth() / 2;
                int y = image.getHeight() / 2;
                sl::float4 point_cloud_value;
                point_cloud.getValue(x, y, &point_cloud_value);

                geometry_msgs::msg::Twist twist_msg; // Message to be published

                if (std::isfinite(point_cloud_value.z))
                {
                    float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                    RCLCPP_INFO(this->get_logger(), "Distance to Camera at {%d; %d}: %f mm", x, y, distance);

                    if (distance < 1000)
                    {
                        RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
                        // Set linear and angular velocities to 0
                        twist_msg.linear.x = 0.0;
                        twist_msg.angular.z = 0.0;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Path is clear. Moving forward.");
                        // Set a forward linear velocity, and no rotation
                        twist_msg.linear.x = 0.5; // Example speed
                        twist_msg.angular.z = 0.0;
                    }

                    pub_->publish(twist_msg); // Publish the message
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "The distance cannot be computed at {%d; %d}", x, y);
                }
                i++;
            }
        }
        zed.close();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
