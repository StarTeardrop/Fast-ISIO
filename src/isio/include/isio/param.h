#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer
{
public:
    ros::NodeHandle nh;

    string sonarPointCloudTopic; 
    string sonarImageTopic;      
    string imuTopic;            
    string odomTopic;           
    string poseGtTopic;         


    string sonarFrame;   
    string baselinkFrame; 
    string odometryFrame;
    string mapFrame;     


    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    float addGpsInterval;

    int N_SCAN;         
    int Horizon_SCAN;   
    int downsampleRate;  
    float sonarMinRange; 
    float sonarMaxRange; 

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;

    // feature
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    int numberOfCores;
    double mappingProcessInterval;

    float z_tollerance;
    float rotation_tollerance;

    // Loop closure
    bool loopClosureEnableFlag;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    float imuAccNoise; 
    float imuGyrNoise; 
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity; 
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;   
    Eigen::Matrix3d extRPY;   
    Eigen::Vector3d extTrans; 
    Eigen::Quaterniond extQRPY;

    ParamServer()
    {
        nh.param<std::string>("fast_isio/sonarPointCloudTopic", sonarPointCloudTopic, "/rexrov1/sonar");
        nh.param<std::string>("fast_isio/sonarImageTopic", sonarImageTopic, "/rexrov1/blueview_p900/sonar_image_new");
        nh.param<std::string>("fast_isio/imuTopic", imuTopic, "/rexrov1/imu");
        nh.param<std::string>("fast_isio/odomTopic", odomTopic, "/odomtry/imu");
        nh.param<std::string>("fast_isio/poseGtTopic", poseGtTopic, "/rexrov1/pose_gt");

        nh.param<std::string>("fast_isio/lidarFrame", sonarFrame, "sonar_link");
        nh.param<std::string>("fast_isio/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("fast_isio/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("fast_isio/mapFrame", mapFrame, "map");

        nh.param<bool>("fast_isio/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("fast_isio/useGpsElevation", useGpsElevation, false);
        nh.param<float>("fast_isio/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("fast_isio/poseCovThreshold", poseCovThreshold, 25.0);
        nh.param<float>("fast_isio/addGpsInterval", addGpsInterval, 2.0);

        nh.param<float>("fast_isio/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("fast_isio/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("fast_isio/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("fast_isio/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("fast_isio/imuGravity", imuGravity, 9.80511);
        nh.param<float>("fast_isio/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("fast_isio/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("fast_isio/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("fast_isio/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<int>("fast_isio/N_SCAN", N_SCAN, 256);
        nh.param<int>("fast_isio/Horizon_SCAN", Horizon_SCAN, 512);
        nh.param<int>("fast_isio/downsampleRate", downsampleRate, 1);
        nh.param<float>("fast_isio/sonarMinRange", sonarMinRange, 0.1);
        nh.param<float>("fast_isio/sonarMaxRange", sonarMaxRange, 9);

        nh.param<float>("fast_isio/odometrySurfLeafSize", odometrySurfLeafSize, 0.1);
        nh.param<float>("fast_isio/mappingCornerLeafSize", mappingCornerLeafSize, 0.1);
        nh.param<float>("fast_isio/mappingSurfLeafSize", mappingSurfLeafSize, 0.1);

        nh.param<float>("fast_isio/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("fast_isio/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("fast_isio/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("fast_isio/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("fast_isio/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("fast_isio/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("fast_isio/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("fast_isio/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 10.0);

        nh.param<int>("fast_isio/numberOfCores", numberOfCores, 2);
        nh.param<double>("fast_isio/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("fast_isio/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("fast_isio/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<bool>("fast_isio/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("fast_isio/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("fast_isio/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("fast_isio/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("fast_isio/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("fast_isio/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("fast_isio/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("fast_isio/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("fast_isio/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("fast_isio/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        this_thread::sleep_for(chrono::microseconds(100));
    }


    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // RPY
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}


template <typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template <typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}