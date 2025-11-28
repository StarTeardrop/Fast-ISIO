#include "isio/param.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::X;
// Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::V;
// Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::B;


class ImuPreintegration : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;


    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;


    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;


    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;


    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;


    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;
    int key = 1;


    gtsam::Pose3 imu2Sonar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 sonar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    ImuPreintegration()
    {
      
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImuPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
       
        subOdometry = nh.subscribe<nav_msgs::Odometry>("fast_isio/mapping/odometry_incremental", 5, &ImuPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);


        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); 
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);    
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);        
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); 
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                              
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                            
                                                                                                                                  
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); 


    void imuHandler(const sensor_msgs::Imu::ConstPtr &imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        if (
            !std::isfinite(thisImu.linear_acceleration.x) ||
            !std::isfinite(thisImu.linear_acceleration.y) ||
            !std::isfinite(thisImu.linear_acceleration.z) ||
            !std::isfinite(thisImu.angular_velocity.x) ||
            !std::isfinite(thisImu.angular_velocity.y) ||
            !std::isfinite(thisImu.angular_velocity.z))
        {
            ROS_WARN_STREAM("[IMU] Invalid data (NaN or Inf) detected. Skipping this IMU message. "
                            << "acc: [" << thisImu.linear_acceleration.x << ", "
                            << thisImu.linear_acceleration.y << ", "
                            << thisImu.linear_acceleration.z << "], gyr: ["
                            << thisImu.angular_velocity.x << ", "
                            << thisImu.angular_velocity.y << ", "
                            << thisImu.angular_velocity.z << "]");
            return; 
        }

        // TODO: 
        // if (
        //     std::abs(thisImu.linear_acceleration.x) > 60 ||
        //     std::abs(thisImu.linear_acceleration.y) > 60 ||
        //     std::abs(thisImu.linear_acceleration.z) > 60 ||
        //     std::abs(thisImu.angular_velocity.x) > 2 ||
        //     std::abs(thisImu.angular_velocity.y) > 2 ||
        //     std::abs(thisImu.angular_velocity.z) > 2)
        // {
        //     ROS_WARN("[IMU] Abnormal IMU value detected: acc(%.2f, %.2f, %.2f), gyr(%.2f, %.2f, %.2f)",
        //              thisImu.linear_acceleration.x,
        //              thisImu.linear_acceleration.y,
        //              thisImu.linear_acceleration.z,
        //              thisImu.angular_velocity.x,
        //              thisImu.angular_velocity.y,
        //              thisImu.angular_velocity.z);
        //     return;
        // }

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);


        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;


        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);


        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);


        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";


        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 sonarPose = imuPose.compose(imu2Sonar);
        odometry.pose.pose.position.x = sonarPose.translation().x();
        odometry.pose.pose.position.y = sonarPose.translation().y();
        odometry.pose.pose.position.z = sonarPose.translation().z();
        odometry.pose.pose.orientation.x = sonarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = sonarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = sonarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = sonarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }


    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        if (imuQueOpt.empty())
            return;


        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;

        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 sonarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        if (systemInitialized == false)
        {

            resetOptimization();

            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }


            prevPose_ = sonarPose.compose(sonar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);


            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);


            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);


            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);


            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();


            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            return;
        }

        if (key == 100)
        {
      
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
      
            resetOptimization();
         
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
       
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
    
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
         
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
        
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

       
        while (!imuQueOpt.empty())
        {
           
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
        
                imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuT_opt = imuTime;
          
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
  
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        gtsam::Pose3 curPose = sonarPose.compose(sonar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
  
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
   
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
  
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
 
        gtsam::Values result = optimizer.calculateEstimate();

        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));

        prevState_ = gtsam::NavState(prevPose_, prevVel_);

        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));

        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }

        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;
 
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }

        if (!imuQueImu.empty())
        {

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);

            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }


    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }


    bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 0.6)    // 0.6
        {
            ROS_WARN("Reset IMU Preintegration...");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }
};


class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subSonarOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f sonarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform sonar2Baselink;

    double sonarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion()
    {

        if (sonarFrame != baselinkFrame)
        {

            tf::Vector3 translation(extTrans[0], extTrans[1], extTrans[2]);
            sonar2Baselink.setOrigin(translation);


            tf::Quaternion rotation(0.0, 0.0, 0.0, 1.0);
            sonar2Baselink.setRotation(rotation);
        }

        subSonarOdometry = nh.subscribe<nav_msgs::Odometry>("fast_isio/mapping/odometry", 5, &TransformFusion::sonarOdometryHandler, this, ros::TransportHints().tcpNoDelay());

        subImuOdometry = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);

        pubImuPath = nh.advertise<nav_msgs::Path>("fast_isio/imu/path", 1);
    }


    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {

        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        if (sonarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= sonarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }

        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());

        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());

        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;

        Eigen::Affine3f imuOdomAffineLast = sonarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

        nav_msgs::Odometry sonarOdometry = imuOdomQueue.back();
        sonarOdometry.pose.pose.position.x = x;
        sonarOdometry.pose.pose.position.y = y;
        sonarOdometry.pose.pose.position.z = z;
        sonarOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(sonarOdometry);


        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(sonarOdometry.pose.pose, tCur);

        if (sonarFrame != baselinkFrame)
            tCur = tCur * sonar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

 
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();


        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = sonarOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);

            while (!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < sonarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }


    void sonarOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sonarOdomAffine = odom2affine(*odomMsg);

        sonarOdomTime = odomMsg->header.stamp.toSec();
    }


    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fast_isio_imu_preintegration");

    ImuPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> Imu Preintegration Started.\033[0m");

    ros::MultiThreadedSpinner spinner(4);

    spinner.spin();

    return 0;
}