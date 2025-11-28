#include "isio/param.h"
#include "isio/image_sonar_cloud_info.h"
#include <pcl/filters/filter.h>


struct VelodynePointXYZIRT
{

    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;

    uint16_t ring;

    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16; 

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

using PointXYZIRT = VelodynePointXYZIRT;


const int queueLength = 2000;


class SonarProcess : public ParamServer
{
public:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subSonarCloud;

    ros::Publisher pubSonarCloud;

    ros::Publisher pubSonarCloudInfo;

    ros::Publisher pubExtractedCloud;


    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;


    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;


    std::deque<sensor_msgs::PointCloud2> cloudQueue;


    sensor_msgs::PointCloud2 currentCloudMsg;


    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;


    pcl::PointCloud<PointXYZIRT>::Ptr sonarCloudIn;

    pcl::PointCloud<PointType>::Ptr fullCloud;

    pcl::PointCloud<PointType>::Ptr extractedCloud;

    PointXYZIRT invalid_point_; 

    int deskewFlag = 0;
    cv::Mat rangeMat;

    bool odomDeskewFlag;

    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;


    isio::image_sonar_cloud_info cloudInfo;

    double timeScanCur;

    double timeScanEnd;

    std_msgs::Header cloudHeader;

    SonarProcess()
    {
       
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &SonarProcess::imuHandler, this, ros::TransportHints().tcpNoDelay());
 
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &SonarProcess::odometryHandler, this, ros::TransportHints().tcpNoDelay());
      
        subSonarCloud = nh.subscribe<sensor_msgs::LaserScan>(sonarPointCloudTopic, 5, &SonarProcess::cloudHandler, this, ros::TransportHints().tcpNoDelay());
      
        pubSonarCloud = nh.advertise<sensor_msgs::PointCloud2>("fast_isio/sonar/cloud", 100);
     
        pubSonarCloudInfo = nh.advertise<isio::image_sonar_cloud_info>("fast_isio/sonar/cloud_info", 100);

      
        allocateMemory();
     
        resetParameters();

      
        invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
        invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
        invalid_point_.z = std::numeric_limits<float>::quiet_NaN();
    }


    void allocateMemory()
    {
        sonarCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);
        resetParameters();
    }


    void resetParameters()
    {
        sonarCloudIn->clear();
        extractedCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }


    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
       
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }


    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::LaserScanConstPtr &sonarCloudMsg)
    {
       
        if (!cachePointCloud(sonarCloudMsg))
            return;

       
        if (!deskewInfo())
            return;

      
        projectPointCloud();

        
        cloudExtraction();

        
        publishClouds();

       
        resetParameters();
    }


    bool cachePointCloud(const sensor_msgs::LaserScanConstPtr &sonarCloudMsg)
    {
        pcl::PointCloud<PointXYZIRT>::Ptr cloud_msg = boost::make_shared<pcl::PointCloud<PointXYZIRT>>();

        int vertical_beams = N_SCAN;                                   
        float vertical_fov = 20.0 * M_PI / 180.0;                       
        float vertical_angle_step = vertical_fov / (vertical_beams - 1); 

        float stack_height = sonarMaxRange * sin(vertical_fov / 2.0f);

        int horizontal_beams = sonarCloudMsg->ranges.size();
        cloud_msg->points.resize(horizontal_beams * vertical_beams);
        for (unsigned int i = 0; i < sonarCloudMsg->ranges.size(); ++i)
        {

            float range = sonarCloudMsg->ranges[i];
            if (range >= sonarMaxRange || range <= 0)
            {
                for (int j = 0; j < vertical_beams; ++j)
                {
                    int idx = i * vertical_beams + j;
                    cloud_msg->points[idx] = invalid_point_; 
                    cloud_msg->points[idx].ring = j;         
                }
                continue;
            }

            float angle_horizontal = sonarCloudMsg->angle_min + i * sonarCloudMsg->angle_increment;

            for (int j = 0; j < vertical_beams; ++j)
            {
                int idx = i * vertical_beams + j;
                PointXYZIRT &point_tmp = cloud_msg->points[idx];

                float angle_vertical = j * vertical_angle_step;

                point_tmp.x = range * cos(angle_vertical) * cos(angle_horizontal);
                point_tmp.y = range * cos(angle_vertical) * sin(angle_horizontal);


                point_tmp.z = stack_height * sin(angle_vertical);


                point_tmp.ring = j; 

                point_tmp.intensity = range;
                float time_offset = float(i) / float(horizontal_beams);             
                point_tmp.time = sonarCloudMsg->header.stamp.toSec() + time_offset; 
            }
        }

        cloud_msg->width = sonarCloudMsg->ranges.size();
        cloud_msg->height = vertical_beams;
        cloud_msg->is_dense = true; 


        sensor_msgs::PointCloud2 output_cloud_msg;
        pcl::toROSMsg(*cloud_msg, output_cloud_msg);

        output_cloud_msg.header = sonarCloudMsg->header;

        cloudQueue.push_back(output_cloud_msg);
        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();


        pcl::moveFromROSMsg(currentCloudMsg, *sonarCloudIn);


        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + 0.01;


        if (sonarCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        return true;
    }


    void projectPointCloud()
    {
        int cloudSize = sonarCloudIn->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = sonarCloudIn->points[i].x;
            thisPoint.y = sonarCloudIn->points[i].y;
            thisPoint.z = sonarCloudIn->points[i].z;
            thisPoint.intensity = sonarCloudIn->points[i].intensity;

            if (std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z))
                continue;


            float range = pointDistance(thisPoint);
            if (range < sonarMinRange || range > sonarMaxRange)
                continue;

            int rowIdn = sonarCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;


            if (rowIdn % downsampleRate != 0)
                continue;


            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;


            static float ang_res_x = 130.0 / float(Horizon_SCAN);

            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;

            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;


            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }


    void cloudExtraction()
    {

        int count = 0;

        for (int i = 0; i < N_SCAN; ++i)
        {
 
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
   
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                   
                    cloudInfo.pointColInd[count] = j;
            
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    ++count;
                }
            }
            
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }


    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

     
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            return false;
        }

 
        imuDeskewInfo();


        odomDeskewInfo();

        return true;
    }


    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;
    
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

  
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

   
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

 
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }
        --imuPointerCur;

    
        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }


    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;
     
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

     
        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

   
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

      
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        cloudInfo.odomAvailable = true;

        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

      
        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

      
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

  
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }


    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud = publishCloud(&pubSonarCloud, extractedCloud, cloudHeader.stamp, sonarFrame);
        pubSonarCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_sio_sonar_process");

    SonarProcess SP;

    ROS_INFO("\033[1;32m----> Sonar Process Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);

    spinner.spin();

    return 0;
}