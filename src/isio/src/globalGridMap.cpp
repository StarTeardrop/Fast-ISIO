#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <angles/angles.h>
#include <cmath>

class GlobalGridMap
{
public:
    GlobalGridMap() : nh_("~")
    {
        nh_.param("resolution", resolution_, 0.2f);
        nh_.param("width", width_, 500);
        nh_.param("height", height_, 500);
        nh_.param("origin_x", origin_x_, -50.0f);
        nh_.param("origin_y", origin_y_, -50.0f);
        nh_.param("fov_deg", fov_deg_, 130.0f);
        nh_.param("max_range", max_range_, 10.0f);
        nh_.param("unknown_area_color", unknown_area_color, -1);
        nh_.param("explored_area_color", explored_area_color, 0);
        nh_.param("obstacle_color", obstacle_color, 100);
        ROS_INFO("Grid Map: resolution: %f", resolution_);
        ROS_INFO("Grid Map: width: %d", width_);
        ROS_INFO("Grid Map: height: %d", height_);
        ROS_INFO("Grid Map: origin_x: %f", origin_x_);
        ROS_INFO("Grid Map: origin_y: %f", origin_y_);
        ROS_INFO("Grid Map: fov_deg: %f", fov_deg_);
        ROS_INFO("Grid Map: max_range: %f", max_range_);
        ROS_INFO("Grid Map: unknown_area_color: %d", unknown_area_color);
        ROS_INFO("Grid Map: explored_area_color: %d", explored_area_color);
        ROS_INFO("Grid Map: obstacle_color: %d", obstacle_color);

        global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        path_sub_ = nh_.subscribe("/fast_isio/mapping/path", 10, &GlobalGridMap::pathCallback, this);
        cloud_sub_ = nh_.subscribe("/fast_isio/mapping/map_global", 10, &GlobalGridMap::pointCloudCallback, this);
        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/fast_isio/mapping/global_grid_map", 10);
    }

private:
    float resolution_;
    int width_;
    int height_;
    float origin_x_;
    float origin_y_;
    float fov_deg_;
    float max_range_;

    int unknown_area_color; 
    int explored_area_color; 
    int obstacle_color;      

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher grid_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;

public:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::fromROSMsg(*msg, *global_cloud_);
    }

    bool pointInFOV(float px, float py, float yaw, float tx, float ty, float fov_deg, float max_range)
    {
        float dx = tx - px;
        float dy = ty - py;
        float r = std::hypot(dx, dy);
        if (r > max_range)
            return false;

        float angle_to_point = std::atan2(dy, dx);
        float delta_angle = angles::shortest_angular_distance(yaw, angle_to_point);
        return std::abs(delta_angle) <= fov_deg * 0.5 * M_PI / 180.0;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &path)
    {

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto &pose_stamped : path->poses)
        {
            float x = pose_stamped.pose.position.x;
            float y = pose_stamped.pose.position.y;
            min_x = std::min(min_x, x - max_range_);
            max_x = std::max(max_x, x + max_range_);
            min_y = std::min(min_y, y - max_range_);
            max_y = std::max(max_y, y + max_range_);
        }


        origin_x_ = min_x;
        origin_y_ = min_y;
        width_ = std::ceil((max_x - min_x) / resolution_);
        height_ = std::ceil((max_y - min_y) / resolution_);

        // ROS_INFO_THROTTLE(1.0, "Updated Grid Map: origin(%.2f, %.2f), size=(%d x %d), resolution=%.3f",
        //                   origin_x_, origin_y_, width_, height_, resolution_);

        std::vector<int8_t> grid_map(width_ * height_, unknown_area_color); 

#pragma omp parallel for num_threads(4)

        for (const auto &pose_stamped : path->poses)
        {
            float px = pose_stamped.pose.position.x;
            float py = pose_stamped.pose.position.y;

            tf::Quaternion q(
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            int num_rays = 512;
            for (int i = 0; i < num_rays; ++i)
            {
                float angle = yaw - (fov_deg_ * 0.5 * M_PI / 180.0) + (fov_deg_ * M_PI / 180.0) * i / (num_rays - 1);
                for (float r = 0; r <= max_range_; r += resolution_)
                {
                    float rx = px + r * std::cos(angle);
                    float ry = py + r * std::sin(angle);
                    int gx = static_cast<int>((rx - origin_x_) / resolution_);
                    int gy = static_cast<int>((ry - origin_y_) / resolution_);
                    if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_)
                    {
                        int idx = gy * width_ + gx;
                        if (grid_map[idx] == unknown_area_color)
                            grid_map[idx] = explored_area_color; 
                    }
                }
            }

            for (const auto &pt : global_cloud_->points)
            {
                if (pointInFOV(px, py, yaw, pt.x, pt.y, fov_deg_, max_range_))
                {
                    int gx = static_cast<int>((pt.x - origin_x_) / resolution_);
                    int gy = static_cast<int>((pt.y - origin_y_) / resolution_);
                    if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_)
                    {
                        int idx = gy * width_ + gx;
                        grid_map[idx] = obstacle_color; 
                    }
                }
            }
        }

        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = width_;
        grid_msg.info.height = height_;
        grid_msg.info.origin.position.x = origin_x_;
        grid_msg.info.origin.position.y = origin_y_;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data = grid_map;

        grid_pub_.publish(grid_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_grid_map_node");

    GlobalGridMap gridMap;

    ROS_INFO("\033[1;32m----> Global Grid Map Started.\033[0m");

    ros::MultiThreadedSpinner spinner(4);

    spinner.spin();

    return 0;
}
