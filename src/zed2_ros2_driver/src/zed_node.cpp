#include "rclcpp/rclcpp.hpp"
#include <sl/Camera.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <mutex>

using namespace sl;

enum VoxelState
{
    UNKNOWN,
    FREE,
    OCCUPIED
};

const int GRID_SIZE_X = 1000;
const int GRID_SIZE_Y = 500;
const int GRID_SIZE_Z = 400;
const float VOXEL_RESOLUTION = 0.1;
const float NAVIGABLE_THRESHOLD = 0.2;
const float ORIGIN_OFFSET_X = -GRID_SIZE_X * VOXEL_RESOLUTION * 0.5;
const float ORIGIN_OFFSET_Y = -GRID_SIZE_Y * VOXEL_RESOLUTION * 0.5;
const float ORIGIN_OFFSET_Z = 0;

class ZedNode : public rclcpp::Node
{
public:
    ZedNode() : Node("zed_node")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        voxelGrid.resize(GRID_SIZE_X);
        for (int x = 0; x < GRID_SIZE_X; x++)
        {
            voxelGrid[x].resize(GRID_SIZE_Y);
            for (int y = 0; y < GRID_SIZE_Y; y++)
            {
                voxelGrid[x][y].resize(GRID_SIZE_Z, UNKNOWN);
            }
        }

        detectObstacle();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::vector<std::vector<std::vector<VoxelState>>> voxelGrid;
    std::vector<std::vector<std::vector<int>>> labels;
    std::mutex voxelGridMutex_;

    void detectObstacle()
    {
        Camera zed;
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
                RCLCPP_INFO(this->get_logger(), "Frame grabbed successfully.");
                zed.retrieveImage(image, VIEW::LEFT);
                zed.retrieveMeasure(depth, MEASURE::DEPTH);
                zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

                for (int x = 0; x < GRID_SIZE_X; x++)
                {
                    for (int z = 0; z < GRID_SIZE_Z; z++)
                    {
                        voxelGrid[x][GRID_SIZE_Y - 1][z] = UNKNOWN;
                    }
                }

                int validPoints = updateVoxelGrid(point_cloud);
                estimateObjectSizes();

                int occupiedCount = 0;
                for (int x = 0; x < GRID_SIZE_X; x++)
                {
                    for (int z = 0; z < GRID_SIZE_Z; z++)
                    {
                        if (voxelGrid[x][GRID_SIZE_Y - 1][z] == OCCUPIED)
                        {
                            occupiedCount++;
                        }
                    }
                }
                RCLCPP_INFO(this->get_logger(), "Number of OCCUPIED voxels in front: %d", occupiedCount);

                if (isTerrainNavigable(validPoints))
                {
                    RCLCPP_INFO(this->get_logger(), "Terrain is navigable.");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Terrain is not navigable.");
                }

                geometry_msgs::msg::Twist cmd_msg;

                if (isTerrainNavigable(validPoints))
                {
                    cmd_msg.linear.x = 0.5;
                    cmd_msg.angular.z = 0.0;
                }
                else
                {
                    cmd_msg.linear.x = 0.0;
                    cmd_msg.angular.z = 0.0;
                }

                pub_->publish(cmd_msg);
            }
        }

        zed.close();
    }

    int updateVoxelGrid(const sl::Mat &pointCloud)
    {
        int width = pointCloud.getWidth();
        int height = pointCloud.getHeight();
        int validPoints = 0;
        int pointsWithinGrid = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                sl::float4 point;
                pointCloud.getValue(x, y, &point);

                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
                {
                    validPoints++;

                    int voxel_x = (point.x / 1000 - ORIGIN_OFFSET_X) / VOXEL_RESOLUTION;
                    int voxel_y = (point.y / 1000 - ORIGIN_OFFSET_Y) / VOXEL_RESOLUTION;
                    int voxel_z = (point.z / 1000 - ORIGIN_OFFSET_Z) / VOXEL_RESOLUTION;

                    if (0 <= voxel_x && voxel_x < GRID_SIZE_X &&
                        0 <= voxel_y && voxel_y < GRID_SIZE_Y &&
                        0 <= voxel_z && voxel_z < GRID_SIZE_Z)
                    {
                        voxelGrid[voxel_x][voxel_y][voxel_z] = OCCUPIED;
                        pointsWithinGrid++;
                    }

                    // Print a few transformed points for diagnostic purposes
                    if (x % 100 == 0 && y % 100 == 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Point (%f, %f, %f) -> Voxel (%d, %d, %d)",
                                    point.x, point.y, point.z, voxel_x, voxel_y, voxel_z);
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Processed %d points. Valid points: %d. Points within grid: %d",
                    width * height, validPoints, pointsWithinGrid);

        return validPoints;
    }

    bool isTerrainNavigable(int validPoints)
    {
        // If no valid points detected, terrain is not navigable.
        if (validPoints == 0)
            return false;

        int occupiedCount = 0;
        for (int x = 0; x < GRID_SIZE_X; x++)
        {
            for (int z = 0; z < GRID_SIZE_Z; z++)
            {
                if (voxelGrid[x][GRID_SIZE_Y - 1][z] == OCCUPIED)
                {
                    occupiedCount++;
                }
            }
        }

        float occupiedRatio = static_cast<float>(occupiedCount) / (GRID_SIZE_X * GRID_SIZE_Z);
        return occupiedRatio < NAVIGABLE_THRESHOLD;
    }

    void initializeLabels()
    {
        labels.resize(GRID_SIZE_X);
        for (int x = 0; x < GRID_SIZE_X; x++)
        {
            labels[x].resize(GRID_SIZE_Y);
            for (int y = 0; y < GRID_SIZE_Y; y++)
            {
                labels[x][y].resize(GRID_SIZE_Z, 0);
            }
        }
    }

    void DFS(int x, int y, int z, int currentLabel)
    {
        if (x < 0 || x >= GRID_SIZE_X || y < 0 || y >= GRID_SIZE_Y || z < 0 || z >= GRID_SIZE_Z)
            return;

        if (voxelGrid[x][y][z] != OCCUPIED || labels[x][y][z] != 0)
            return;

        labels[x][y][z] = currentLabel;

        DFS(x + 1, y, z, currentLabel);
        DFS(x - 1, y, z, currentLabel);
        DFS(x, y + 1, z, currentLabel);
        DFS(x, y - 1, z, currentLabel);
        DFS(x, y, z + 1, currentLabel);
        DFS(x, y, z - 1, currentLabel);
    }

    void estimateObjectSizes()
    {
        initializeLabels();

        int currentLabel = 1;
        for (int x = 0; x < GRID_SIZE_X; x++)
        {
            for (int y = 0; y < GRID_SIZE_Y; y++)
            {
                for (int z = 0; z < GRID_SIZE_Z; z++)
                {
                    if (voxelGrid[x][y][z] == OCCUPIED && labels[x][y][z] == 0)
                    {
                        DFS(x, y, z, currentLabel);
                        currentLabel++;
                    }
                }
            }
        }

        for (int label = 1; label < currentLabel; label++)
        {
            int minX = GRID_SIZE_X, minY = GRID_SIZE_Y, minZ = GRID_SIZE_Z;
            int maxX = 0, maxY = 0, maxZ = 0;

            for (int x = 0; x < GRID_SIZE_X; x++)
            {
                for (int y = 0; y < GRID_SIZE_Y; y++)
                {
                    for (int z = 0; z < GRID_SIZE_Z; z++)
                    {
                        if (labels[x][y][z] == label)
                        {
                            minX = std::min(minX, x);
                            minY = std::min(minY, y);
                            minZ = std::min(minZ, z);
                            maxX = std::max(maxX, x);
                            maxY = std::max(maxY, y);
                            maxZ = std::max(maxZ, z);
                        }
                    }
                }
            }

            float objectWidth = (maxX - minX + 1) * VOXEL_RESOLUTION;
            float objectHeight = (maxY - minY + 1) * VOXEL_RESOLUTION;
            float objectDepth = (maxZ - minZ + 1) * VOXEL_RESOLUTION;

            RCLCPP_INFO(this->get_logger(), "Object %d: Width: %f, Height: %f, Depth: %f", label, objectWidth, objectHeight, objectDepth);
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
