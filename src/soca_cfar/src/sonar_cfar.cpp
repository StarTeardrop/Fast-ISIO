#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <cfar.h>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


struct VelodynePointXYZIRT
{
  
    PCL_ADD_POINT4D
  
    PCL_ADD_INTENSITY;

    std::uint16_t ring;
 
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))


using PointXYZIRT = VelodynePointXYZIRT;

class SonarCFARProcessor
{
public:
    SonarCFARProcessor() : nh_("~")
    {

        nh_.param("train_cells", train_cells_, 12);
        nh_.param("guard_cells", guard_cells_, 4);
        nh_.param("Pfa", Pfa_, 0.6f);
        ROS_INFO("Loaded cfar parameters: train_cells=%d, guard_cells=%d, Pfa=%.3f", train_cells_, guard_cells_, Pfa_);
        cfar_ = CFAR(train_cells_, guard_cells_, Pfa_);


        nh_.param("sonar_image_width", sonar_image_width_, 512);
        nh_.param("sonar_image_height", sonar_image_height_, 312);
        nh_.param("sonar_image_margin_ratio", sonar_image_margin_ratio_, 0.05f);
        ROS_INFO("Loaded sonar image parameters: width=%d, height=%d, margin_ratio=%.2f", sonar_image_width_, sonar_image_height_, sonar_image_margin_ratio_);


        nh_.param("sonar_horizon_fov", sonar_horizon_fov_, 130.0f);
        nh_.param("sonar_vertical_fov", sonar_vertical_fov_, 20.0f);
        ROS_INFO("Loaded sonar horizon fov: %.2f, vertical fov: %.2f", sonar_horizon_fov_, sonar_vertical_fov_);


        nh_.param("adaptive_threshold_value", adaptive_threshold_value, 70);
        ROS_INFO("Loaded adaptive threshold value: %d", adaptive_threshold_value);
        nh_.param("canny_threshold_low", canny_threshold_low, 30);
        nh_.param("canny_threshold_high", canny_threshold_high, 100);
        ROS_INFO("Loaded canny threshold: low=%d, high=%d", canny_threshold_low, canny_threshold_high);


        nh_.param("sonar_scan_range", sonar_scan_range, 10.0f);
        ROS_INFO("Loaded sonar scan range: %.2f", sonar_scan_range);


        nh_.param("sonar_scan_horizon_beam", sonar_scan_horizon_beam, 512);
        nh_.param("sonar_scan_vertical_beam", sonar_scan_vertical_beam, 256);
        ROS_INFO("Loaded sonar scan beam: horizon=%d, vertical=%d", sonar_scan_horizon_beam, sonar_scan_vertical_beam);


        float angle_deg = 90.0f - sonar_horizon_fov_ / 2.0f;
        float angle_rad = angle_deg * M_PI / 180.0f;
        sonar_image_width_margin_crop_ = int(sonar_image_width_ * sonar_image_margin_ratio_);
        sonar_image_height_margin_crop_ = int((sonar_image_height_ - (sonar_image_width_ / 2 - sonar_image_width_margin_crop_) / cos(angle_rad)) / 2.0f);
        ROS_INFO("Sonar image margin crop: width=%d, height=%d", sonar_image_width_margin_crop_, sonar_image_height_margin_crop_);


        float vertical_fov_rad = sonar_vertical_fov_ / 2.0f * M_PI / 180.0f;
        stack_height = sonar_scan_range * sin(vertical_fov_rad);
        ROS_INFO("Sonar stack height: %.2f m", stack_height);


        nh_.param("voxel_grid_resolution", voxel_grid_resolution, 0.01f);
        ROS_INFO("Loaded voxel grid resolution: %.2f m", voxel_grid_resolution);


        sonar_pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        sonar_edge_pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        sonar_stack_edge_pointcloud = pcl::PointCloud<PointXYZIRT>::Ptr(new pcl::PointCloud<PointXYZIRT>);


        sonar_sub_ = nh_.subscribe("/BlueRov2/0/sonar", 10, &SonarCFARProcessor::sonarCallback, this);
        sonar_cfar_pub_ = nh_.advertise<sensor_msgs::Image>("/BlueRov2/0/sonar_cfar", 10);
        sonar_cfar_crop_pub_ = nh_.advertise<sensor_msgs::Image>("/BlueRov2/0/sonar_cfar_cropped", 10);
        sonar_filtered_pub_ = nh_.advertise<sensor_msgs::Image>("/BlueRov2/0/sonar_final_filtered", 10);
        sonar_outline_pub_ = nh_.advertise<sensor_msgs::Image>("/BlueRov2/0/sonar_outline", 10);
        sonar_origin_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/BlueRov2/0/sonar_origin_pointcloud", 10);
        sonar_edge_feature_image_pub_ = nh_.advertise<sensor_msgs::Image>("/BlueRov2/0/sonar_edge_feature_image", 10);
        sonar_edge_laserscan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/BlueRov2/0/sonar_edge_feature_laserscan", 10);
        sonar_stack_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/BlueRov2/0/sonar_stack_pointcloud", 10);

        sonar_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar_fov_marker", 10);
    }

private:
    int train_cells_;
    int guard_cells_;
    float Pfa_;
    CFAR cfar_;

    int sonar_image_width_;
    int sonar_image_height_;
    float sonar_image_margin_ratio_;

    float sonar_horizon_fov_; 
    float sonar_vertical_fov_; 

    int adaptive_threshold_value; 
    int canny_threshold_low;     
    int canny_threshold_high;    

    float sonar_scan_range; 

    int sonar_scan_horizon_beam;  
    int sonar_scan_vertical_beam; 

    float stack_height; 

    float voxel_grid_resolution; 


    pcl::PointCloud<pcl::PointXYZ>::Ptr sonar_pointcloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sonar_edge_pointcloud;

    pcl::PointCloud<PointXYZIRT>::Ptr sonar_stack_edge_pointcloud;

    int sonar_image_width_margin_crop_;
    int sonar_image_height_margin_crop_;

    ros::NodeHandle nh_;                
    ros::Subscriber sonar_sub_;         
    ros::Publisher sonar_cfar_pub_;     
    ros::Publisher sonar_cfar_crop_pub_; 
    ros::Publisher sonar_filtered_pub_; 
    ros::Publisher sonar_outline_pub_;  

    ros::Publisher sonar_origin_pointcloud_pub_;  
    ros::Publisher sonar_edge_feature_image_pub_; 
    ros::Publisher sonar_edge_laserscan_pub_;     
    ros::Publisher sonar_stack_pointcloud_pub_;   

    ros::Publisher sonar_marker_pub_; 

    void sonarCallback(const sensor_msgs::Image::ConstPtr &msg)
    {
        if (msg->encoding != "mono8")
        {
            ROS_WARN("Only mono8 encoding is supported, got: %s", msg->encoding.c_str());
            return;
        }


        cv::Mat sonar_image(msg->height, msg->width, CV_8UC1, const_cast<uchar *>(&msg->data[0]));


        cv::Mat result_1d = cfar_.soca_1d(sonar_image);


        cv::Mat result_norm;
        cv::normalize(result_1d, result_norm, 0, 255, cv::NORM_MINMAX);
        result_norm.convertTo(result_norm, CV_8UC1);


        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "mono8", result_norm).toImageMsg();
        sonar_cfar_pub_.publish(out_msg);

        int new_width = result_norm.cols - sonar_image_width_margin_crop_ * 2;
        int new_height = result_norm.rows - sonar_image_height_margin_crop_ * 2;
        cv::Rect roi(sonar_image_width_margin_crop_, sonar_image_height_margin_crop_, new_width, new_height);
        cv::Mat cropped_result = result_norm(roi);

        sensor_msgs::ImagePtr crop_msg = cv_bridge::CvImage(msg->header, "mono8", cropped_result).toImageMsg();
        sonar_cfar_crop_pub_.publish(crop_msg);


        int width = cropped_result.size().width;
        int height = cropped_result.size().height;

        cv::Mat sonar_filtered_image;


        cv::threshold(cropped_result, sonar_filtered_image, adaptive_threshold_value, 255, cv::THRESH_BINARY);

        sensor_msgs::ImagePtr threshold_msg = cv_bridge::CvImage(msg->header, "mono8", sonar_filtered_image).toImageMsg();
        sonar_filtered_pub_.publish(threshold_msg);

        float sonar_scan_x = sonar_scan_range * width / height;
        float sonar_scan_y = sonar_scan_range;


        generateSonarPointCloud(sonar_filtered_image, sonar_scan_x, sonar_scan_y, sonar_pointcloud);
        sensor_msgs::PointCloud2 origin_sonar_pointcloud_msg;
        pcl::toROSMsg(*sonar_pointcloud, origin_sonar_pointcloud_msg); 
        origin_sonar_pointcloud_msg.header.stamp = msg->header.stamp;  
        origin_sonar_pointcloud_msg.header.frame_id = "sonar_link";    
        sonar_origin_pointcloud_pub_.publish(origin_sonar_pointcloud_msg);

 
        getSonarImageContourPoints(sonar_filtered_image, sonar_scan_x, sonar_scan_y, sonar_edge_pointcloud, msg);
 
        generateSonarStackEdgePointCloud(sonar_pointcloud, sonar_edge_pointcloud, sonar_stack_edge_pointcloud);
        sensor_msgs::PointCloud2 stack_sonar_pointcloud_msg;
        pcl::toROSMsg(*sonar_stack_edge_pointcloud, stack_sonar_pointcloud_msg); 
        stack_sonar_pointcloud_msg.header.stamp = msg->header.stamp;            
        stack_sonar_pointcloud_msg.header.frame_id = "sonar_link";              
        sonar_stack_pointcloud_pub_.publish(stack_sonar_pointcloud_msg);

        
        generateSonarLaserScanPointCloud(sonar_filtered_image, sonar_scan_x, sonar_scan_y, sonar_horizon_fov_, msg);
    }

    void generateSonarPointCloud(cv::Mat &sonar_image, float sonar_scan_x, float sonar_scan_y, pcl::PointCloud<pcl::PointXYZ>::Ptr &sonar_pointcloud)
    {
      
        sonar_pointcloud->points.clear();
        int width = sonar_image.cols;
        int height = sonar_image.rows;

       
        float meter_per_pixel_x = sonar_scan_y / height; 
        float meter_per_pixel_y = sonar_scan_x / width;  

        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                uchar pixel = sonar_image.at<uchar>(v, u);
                if (pixel > 0)
                {
                  
                    float dx = (height - 1 - v);  
                    float dy = (u - width / 2.0f); 

                    float x = dx * meter_per_pixel_x; 
                    float y = -dy * meter_per_pixel_y;
                    float z = 0.0f;

                    sonar_pointcloud->points.emplace_back(x, y, z);
                }
            }
        }
        sonar_pointcloud->width = sonar_pointcloud->points.size();
        sonar_pointcloud->height = 1;
        sonar_pointcloud->is_dense = true;
    }

    void generateSonarLaserScanPointCloud(cv::Mat &sonar_image, float sonar_scan_x, float sonar_scan_y, int horizontal_fov, const sensor_msgs::Image::ConstPtr &msg)
    {
        int width = sonar_image.cols;
        int height = sonar_image.rows;


        float origin_x = width / 2.0f;
        float origin_y = height - 1; 

        int num_rays = sonar_scan_horizon_beam;

        float angle_start = -horizontal_fov / 2.0f;
        float angle_increment = (float)horizontal_fov / num_rays;


        cv::Mat color_image;
        cv::cvtColor(sonar_image, color_image, cv::COLOR_GRAY2BGR);


        int hit_count = 0;


        sensor_msgs::LaserScan laser_scan;
        laser_scan.header = msg->header;
        laser_scan.header.frame_id = "sonar_link";                             
        laser_scan.angle_min = angle_start * CV_PI / 180.0f;                    
        laser_scan.angle_max = (angle_start + horizontal_fov) * CV_PI / 180.0f; 
        laser_scan.angle_increment = angle_increment * CV_PI / 180.0f;         
        laser_scan.time_increment = 0.0;                                        
        laser_scan.scan_time = 0.1;                                            
        laser_scan.range_min = 0.1;                                             
        laser_scan.range_max = sonar_scan_range;                               


        laser_scan.ranges.resize(num_rays, laser_scan.range_max);
        laser_scan.intensities.resize(num_rays, 0.0);


        float pixel_to_meter = sonar_scan_range / height;

        for (int i = 0; i < num_rays; ++i)
        {

            float angle_deg = angle_start + (num_rays - 1 - i) * angle_increment;
            float angle_rad = angle_deg * CV_PI / 180.0f;

            float dx = std::sin(angle_rad);
            float dy = -std::cos(angle_rad); 


            bool hit = false;
            for (float step = 0.0f; step < std::max(width, height); step += 0.25f) 
            {
                float x = origin_x + dx * step;
                float y = origin_y + dy * step;

                int xi = static_cast<int>(std::round(x));
                int yi = static_cast<int>(std::round(y));

                if (xi < 0 || xi >= width || yi < 0 || yi >= height)
                    break;

                if (sonar_image.at<uchar>(yi, xi) > 15)
                {
                    cv::circle(color_image, cv::Point(xi, yi), 1, cv::Scalar(0, 0, 255), -1); 


                    float distance_pixels = std::sqrt(std::pow(xi - origin_x, 2) + std::pow(yi - origin_y, 2));

    
                    float distance_meters = distance_pixels * pixel_to_meter;

     
                    if (i < laser_scan.ranges.size())
                    {
                        laser_scan.ranges[i] = distance_meters;
                        laser_scan.intensities[i] = sonar_image.at<uchar>(yi, xi); 
                    }

                    hit = true;
                    hit_count++;
                    break;
                }
            }


            if (!hit)
            {
                int end_x = static_cast<int>(std::round(origin_x + dx * std::max(width, height)));
                int end_y = static_cast<int>(std::round(origin_y + dy * std::max(width, height)));
                cv::line(color_image, cv::Point(origin_x, origin_y), cv::Point(end_x, end_y), cv::Scalar(0, 255, 0), 1);
            }
        }


        sensor_msgs::ImagePtr edge_feature_msg = cv_bridge::CvImage(msg->header, "bgr8", color_image).toImageMsg();
        sonar_edge_feature_image_pub_.publish(edge_feature_msg);

        sonar_edge_laserscan_pub_.publish(laser_scan);


        visualization_msgs::Marker fov_marker;
        fov_marker.header = msg->header;
        fov_marker.header.frame_id = "sonar_link"; /
        fov_marker.ns = "sonar_fov_fill";
        fov_marker.id = 0;
        fov_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        fov_marker.action = visualization_msgs::Marker::ADD;

        fov_marker.scale.x = 1.0;
        fov_marker.scale.y = 1.0;
        fov_marker.scale.z = 1.0;


        fov_marker.pose.position.x = 0.0;
        fov_marker.pose.position.y = 0.0;
        fov_marker.pose.position.z = 0.0;
        fov_marker.pose.orientation.x = 0.0;
        fov_marker.pose.orientation.y = 0.0;
        fov_marker.pose.orientation.z = 0.0;
        fov_marker.pose.orientation.w = 1.0;


        fov_marker.color.a = 0.7; 
        fov_marker.color.r = 0.93; 
        fov_marker.color.g = 0.62; 
        fov_marker.color.b = 0.87; 


        geometry_msgs::Point origin;
        origin.x = 0;
        origin.y = 0;
        origin.z = 0;

        float angle_start_rad = angle_start * CV_PI / 180.0f;
        float angle_inc_rad = angle_increment * CV_PI / 180.0f;

        for (int i = 0; i < num_rays; ++i)
        {
            float angle1 = angle_start_rad + i * angle_inc_rad;
            float angle2 = angle_start_rad + (i + 1) * angle_inc_rad;

            geometry_msgs::Point p1, p2;

            p1.x = sonar_scan_range * cos(angle1);
            p1.y = sonar_scan_range * sin(angle1);
            p1.z = 0;

            p2.x = sonar_scan_range * cos(angle2);
            p2.y = sonar_scan_range * sin(angle2);
            p2.z = 0;


            fov_marker.points.push_back(origin);
            fov_marker.points.push_back(p1);
            fov_marker.points.push_back(p2);
        }


        sonar_marker_pub_.publish(fov_marker);
    }

    void getSonarImageContourPoints(cv::Mat &sonar_image, float sonar_scan_x, float sonar_scan_y,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &contour_points, const sensor_msgs::Image::ConstPtr &msg)
    {
        contour_points->points.clear();
        int width = sonar_image.cols;
        int height = sonar_image.rows;


        float meter_per_pixel_x = sonar_scan_y / height; 
        float meter_per_pixel_y = sonar_scan_x / width;  

        cv::Mat edges;
        int low_threshold = canny_threshold_low;
        int high_threshold = canny_threshold_high;
        cv::Canny(sonar_image, edges, low_threshold, high_threshold);


        cv::Mat color_image;
        cv::cvtColor(sonar_image, color_image, cv::COLOR_GRAY2BGR);
        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                if (edges.at<uchar>(v, u) > 0)
                {
                    color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 255);
                }
            }
        }


        sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(msg->header, "bgr8", color_image).toImageMsg();
        sonar_outline_pub_.publish(contour_msg);


        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                if (edges.at<uchar>(v, u) > 0)
                {
                    float dx = (height - 1 - v);   
                    float dy = (u - width / 2.0f); 

                    float x = dx * meter_per_pixel_x;
                    float y = -dy * meter_per_pixel_y;
                    float z = 0.0f;

                    contour_points->points.emplace_back(x, y, z);
                }
            }
        }

        contour_points->width = contour_points->points.size();
        contour_points->height = 1;
        contour_points->is_dense = true;
    }

    void generateSonarStackEdgePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &sonar_pointcloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &sonar_edge_pointcloud,
                                          pcl::PointCloud<PointXYZIRT>::Ptr &sonar_stack_edge_pointcloud)
    {
        sonar_stack_edge_pointcloud->points.clear();

        int num_layers = int(sonar_scan_vertical_beam / 2.0f); 
        float total_thickness = stack_height;                 
        float delta_z = total_thickness / num_layers;         
        float start_z = 0.0f;


        for (int i = 0; i < num_layers; ++i)
        {
            float z_offset = start_z + i * delta_z;

            if (i == 0 || i == num_layers - 1)
            {

                for (const auto &pt : sonar_pointcloud->points)
                {
                    if (!pcl::isFinite(pt))
                        continue;
                    PointXYZIRT new_pt;
                    new_pt.x = pt.x;
                    new_pt.y = pt.y;
                    new_pt.z = z_offset;
                    new_pt.intensity = std::sqrt(pt.x * pt.x + pt.y * pt.y + z_offset * z_offset);
                    new_pt.ring = i;
                    new_pt.time = 0.0;
                    sonar_stack_edge_pointcloud->points.push_back(new_pt);
                }
            }
            else
            {

                for (const auto &pt : sonar_edge_pointcloud->points)
                {
                    if (!pcl::isFinite(pt))
                        continue;
                    PointXYZIRT new_pt;
                    new_pt.x = pt.x;
                    new_pt.y = pt.y;
                    new_pt.z = z_offset;
                    new_pt.intensity = std::sqrt(pt.x * pt.x + pt.y * pt.y + z_offset * z_offset);
                    new_pt.ring = i;
                    new_pt.time = 0.0;
                    sonar_stack_edge_pointcloud->points.push_back(new_pt);
                }
            }
        }

        sonar_stack_edge_pointcloud->width = sonar_stack_edge_pointcloud->points.size();
        sonar_stack_edge_pointcloud->height = 1;
        sonar_stack_edge_pointcloud->is_dense = true;
    }

    void sonarStackPointCloudFiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr &sonar_stack_edge_pointcloud)
    {

        if (sonar_stack_edge_pointcloud->empty())
            return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(filtered);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f); 
        voxel.filter(*sonar_stack_edge_pointcloud);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_cfar_node");
    ROS_INFO("\033[1;32m----> Sonar CFAR Node Started.\033[0m");
    SonarCFARProcessor processor;
    ros::spin();
    return 0;
}
