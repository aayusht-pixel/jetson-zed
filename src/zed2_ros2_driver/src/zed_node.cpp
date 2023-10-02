#include "rclcpp/rclcpp.hpp"
#include <sl/Camera.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

using namespace sl;

// Define voxel states
enum VoxelState
{
    UNKNOWN,
    FREE,
    OCCUPIED
};

// Define the voxel grid dimensions and resolution
const int GRID_SIZE_X = 50;
const int GRID_SIZE_Y = 50;
const int GRID_SIZE_Z = 50;
const float VOXEL_RESOLUTION = 0.1; // 10cm

class ZedNode : public rclcpp::Node
{
public:
    ZedNode() : Node("zed_node")
    {
        // Initialize a publisher for Twist messages
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize the voxel grid
        voxelGrid.resize(GRID_SIZE_X);
        for (int x = 0; x < GRID_SIZE_X; x++)
        {
            voxelGrid[x].resize(GRID_SIZE_Y);
            for (int y = 0; y < GRID_SIZE_Y; y++)
            {
                voxelGrid[x][y].resize(GRID_SIZE_Z, UNKNOWN);
            }
        }

        // Initialize ZED camera and react based on detected obstacles
        detectObstacle();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::vector<std::vector<std::vector<VoxelState>>> voxelGrid;

    void detectObstacle()
    {
        Camera zed;

        // Set configuration parameters
        InitParameters init_parameters;
        init_parameters.camera_resolution = RESOLUTION::HD720;
        init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
        init_parameters.coordinate_units = UNIT::MILLIMETER;

        auto returned_state = zed.open(init_parameters);
        if (returned_state != ERROR_CODE::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %d: Cannot open ZED camera.", returned_state);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "ZED camera initialized successfully.");
        RCLCPP_INFO(this->get_logger(), "Detecting obstacle...");

        sl::Mat image, depth, point_cloud;

        while (rclcpp::ok())
        {
            if (zed.grab() == ERROR_CODE::SUCCESS)
            {
                zed.retrieveImage(image, VIEW::LEFT);
                zed.retrieveMeasure(depth, MEASURE::DEPTH);
                zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

                // Update the voxel grid with new data
                updateVoxelGrid(point_cloud);

                int x = image.getWidth() / 2;
                int y = image.getHeight() / 2;
                sl::float4 point_cloud_value;
                point_cloud.getValue(x, y, &point_cloud_value);

                geometry_msgs::msg::Twist twist_msg;

                if (std::isfinite(point_cloud_value.z))
                {
                    float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                    RCLCPP_INFO(this->get_logger(), "Distance to Camera at {%d; %d}: %f mm", x, y, distance);

                    if (distance < 1000)
                    {
                        RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
                        twist_msg.linear.x = 0.0;
                        twist_msg.angular.z = 0.0;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Path is clear. Moving forward.");
                        twist_msg.linear.x = 0.5;
                        twist_msg.angular.z = 0.0;
                    }

                    pub_->publish(twist_msg);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "The distance cannot be computed at {%d; %d}", x, y);
                }
            }
        }

        zed.close();
    }

    void updateVoxelGrid(const sl::Mat &pointCloud)
    {
        int width = pointCloud.getWidth();
        int height = pointCloud.getHeight();

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                sl::float4 point;
                pointCloud.getValue(x, y, &point);

                int voxel_x = point.x / VOXEL_RESOLUTION;
                int voxel_y = point.y / VOXEL_RESOLUTION;
                int voxel_z = point.z / VOXEL_RESOLUTION;

                if (0 <= voxel_x && voxel_x < GRID_SIZE_X &&
                    0 <= voxel_y && voxel_y < GRID_SIZE_Y &&
                    0 <= voxel_z && voxel_z < GRID_SIZE_Z)
                {
                    voxelGrid[voxel_x][voxel_y][voxel_z] = OCCUPIED;
                }
            }
        }
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
