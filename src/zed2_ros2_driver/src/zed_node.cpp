#include "rclcpp/rclcpp.hpp"
#include <sl/Camera.hpp>

using namespace sl;

class ZedNode : public rclcpp::Node
{
public:
    ZedNode() : Node("zed_node")
    {
        // Initialize ZED camera and print serial number
        detectObstacle();
    }

private:
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

                if (std::isfinite(point_cloud_value.z))
                {
                    float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                    RCLCPP_INFO(this->get_logger(), "Distance to Camera at {%d; %d}: %f mm", x, y, distance);

                    if (distance < 1000)
                    { // For example, if object is closer than 1 meter
                        RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Path is clear. Moving forward.");
                    }
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
