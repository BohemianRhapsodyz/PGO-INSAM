#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <tf/tf.h>
#include <pcl/point_types.h>
#include <cmath>
#include <deque>
#include "PSINS.h"
#include "KFApp.h"

typedef pcl::PointXYZI PointType;

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


class IMUPreintegration
{
public:

    ros::NodeHandle nh;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;
    bool gpsInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_1;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    CVect3 gps0;
    std::deque<nav_msgs::Odometry> gpsQueue;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    float imuAccNoise=0.01, imuGyrNoise=0.001, imuAccBiasN=0.0002, imuGyrBiasN=0.00003;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    //geometry_msgs::Quaternion qimu2lidar = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -PI/2);
    geometry_msgs::Quaternion qimu2lidar = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    //gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(qimu2lidar.w, qimu2lidar.x, qimu2lidar.y, qimu2lidar.z), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));

    IMUPreintegration()
    {
        subImu      = nh.subscribe<sensor_msgs::Imu>  ("stim300",                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<sensor_msgs::NavSatFix>("novtel", 2000,    &IMUPreintegration::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        //subImu      = nh.subscribe<sensor_msgs::Imu>  ("/udi/imu_s/data",                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        //subOdometry = nh.subscribe<sensor_msgs::NavSatFix>("/novatel718d/pos", 2000,    &IMUPreintegration::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> ("/nav", 2000);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(9.80511);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_1 = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization
    }

    CVect3 lla2enu(double latitude, double longtitude, double altitude, double latitude0, double longtitude0, double altitude0)
    {
        double Re=6378137;
        double f=1/298.257;
        double pi=3.14159265359;
        double deg2rad, e, e2, s1, c1, s12, sq, sq2, RMh, RNh, c1RNh;
        double tmp_latitude0, tmp_longtitude0;
        double x,y,z;
        deg2rad=pi/180;
        tmp_latitude0=latitude0;
        tmp_longtitude0=longtitude0;
        e=sqrt(2*f-f*f);
        e2=e*e;
        s1=sin(latitude);
        c1=cos(latitude);
        s12=s1*s1;
        sq=1-e2*s12;
        sq2=sqrt(sq);
        RMh=Re*(1-e2)/sq/sq2+altitude;
        RNh=Re/sq2+altitude;
        c1RNh=c1*RNh;
        x=(longtitude-tmp_longtitude0)*c1RNh;
        y=(latitude-tmp_latitude0)*RMh;
        z=altitude-altitude0;
        return CVect3(x,y,z);
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

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
    {
        double currentCorrectionTime = ROS_TIME(gpsMsg);

        if (imuQueOpt.empty())
            return;

        //CVect3 enu=lla2enu(PI/180*gpsMsg->latitude, PI/180*gpsMsg->longitude, gpsMsg->altitude, latitude0, longtitude0, altitude0);
        CVect3 enu=lla2enu(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, latitude0, longtitude0, altitude0);
        nav_msgs::Odometry gpsodom;
        gpsodom.header.stamp = gpsMsg->header.stamp;
        gpsodom.pose.pose.position.x = enu.i;
        gpsodom.pose.pose.position.y = enu.j;
        gpsodom.pose.pose.position.z = enu.k;
        //gpsQueue.push_back(gpsodom);

        float p_x = enu.i;
        float p_y = enu.j;
        float p_z = enu.k;

        //quat convert
        Eigen::Vector3d extV(PI/2,0,0);
        Eigen::Matrix3d extRPY1, extRPY2;
        Eigen::Quaterniond extQRPY1, extQRPY2;
        extRPY1 << 1, 0, 0,
                0,-1, 0,
                0, 0, 1;
        extQRPY1 = Eigen::Quaterniond(extRPY1);
//        extRPY2 = Eigen::AngleAxisd(extV[0], Eigen::Vector3d::UnitZ())
//                  *Eigen::AngleAxisd(extV[1], Eigen::Vector3d::UnitY())
//                  *Eigen::AngleAxisd(extV[2], Eigen::Vector3d::UnitX());
        extRPY2 = Eigen::AngleAxisd(extV[0], Eigen::Vector3d::UnitZ());
        //extQRPY2 = Eigen::Quaterniond(extRPY2);
        extQRPY2 = extRPY2;

        //CQuat(CVect3(roll0,pitch0,yaw0));
        //geometry_msgs::Quaternion q0 = tf::createQuaternionMsgFromRollPitchYaw(roll0, -pitch0, (yaw0+PI/2));
        geometry_msgs::Quaternion q0 = tf::createQuaternionMsgFromRollPitchYaw(-roll0, pitch0, (yaw0+PI/2));
        //geometry_msgs::Quaternion q0 = tf::createQuaternionMsgFromRollPitchYaw(roll0, pitch0, yaw0);
        Eigen::Quaterniond q_temp;
        q_temp.x()=q0.x;
        q_temp.y()=q0.y;
        q_temp.z()=q0.z;
        q_temp.w()=q0.w;
        Eigen::Quaterniond q1 = q_temp * extQRPY2;

//        float r_x = q1.x();
//        float r_y = q1.y();
//        float r_z = q1.z();
//        float r_w = q1.w();

        float r_x = q_temp.x();
        float r_y = q_temp.y();
        float r_z = q_temp.z();
        float r_w = q_temp.w();

        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize gps
        if (gpsInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
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
            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;

            gpsInitialized = true;
            //ROS_INFO("initialzed!");
            return;
        }

        // reset graph for speed
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
//                ROS_INFO("dt = %lf",dt);
                if (dt == 0.0)
                    dt = 1.0/500.0;
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add gps factor
        /*gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);*/
        gtsam::Vector Vector3(3);
        Vector3 << 1.0f, 1.0f, 1.0f;
        noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
        gtsam::GPSFactor gps_factor(X(key), gtsam::Point3(p_x, p_y, p_z), gps_noise);
        graphFactors.add(gps_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        // 2. after optimization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
                if (dt == 0.0)
                    dt = 1.0/500.0;
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;

    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
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

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
            ROS_INFO("into imuhandler");
            sensor_msgs::Imu thisImu = *imu_raw;
            //Convert IMU
            thisImu.linear_acceleration.x = imu_raw->linear_acceleration.y;
            thisImu.linear_acceleration.y = -imu_raw->linear_acceleration.x;
            thisImu.angular_velocity.x = imu_raw->angular_velocity.y;
            thisImu.angular_velocity.y = -imu_raw->angular_velocity.x;

            imuQueOpt.push_back(thisImu);
            imuQueImu.push_back(thisImu);

            if (doneFirstOpt == false)
                return;

            double imuTime = ROS_TIME(&thisImu);
            double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
            lastImuT_imu = imuTime;
         if (dt == 0.0)
            dt = 1.0/500.0;
            // integrate this single imu message
            imuIntegratorImu_->integrateMeasurement(
                    gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y,
                                   thisImu.linear_acceleration.z),
                    gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z),
                    dt);

            // predict odometry
            gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
            //quat convert
            Eigen::Vector3d extV(-PI / 2, 0, 0);
            //Eigen::Vector3d extV(0, 0, 0);
            Eigen::Matrix3d extRPY1, extRPY2;
            Eigen::Quaterniond extQRPY1, extQRPY2;
            extRPY1 << 1, 0, 0,
                    0, -1, 0,
                    0, 0, 1;
            extQRPY1 = Eigen::Quaterniond(extRPY1);
            extRPY2 = Eigen::AngleAxisd(extV[0], Eigen::Vector3d::UnitZ())
                      * Eigen::AngleAxisd(extV[1], Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(extV[2], Eigen::Vector3d::UnitX());
            //extQRPY2 = Eigen::Quaterniond(extRPY2);
            extQRPY2 = extRPY2;
            // publish odometry
            nav_msgs::Odometry odometry;
            odometry.header.stamp = thisImu.header.stamp;
            odometry.header.frame_id = "odometryFrame";
            odometry.child_frame_id = "odom_imu";

            // transform imu pose to ldiar
            gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
            gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

            Eigen::Quaterniond q_final = extQRPY2 * lidarPose.rotation().toQuaternion();
            odometry.pose.pose.position.x = lidarPose.translation().x();
            odometry.pose.pose.position.y = lidarPose.translation().y();
            odometry.pose.pose.position.z = lidarPose.translation().z();

            odometry.pose.pose.orientation.x = q_final.x();
            odometry.pose.pose.orientation.y = q_final.y();
            odometry.pose.pose.orientation.z = q_final.z();
            odometry.pose.pose.orientation.w = q_final.w();

            odometry.twist.twist.linear.x = currentState.velocity().x();
            odometry.twist.twist.linear.y = currentState.velocity().y();
            odometry.twist.twist.linear.z = currentState.velocity().z();
            odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
            odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
            odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
            pubImuOdometry.publish(odometry);

    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "od_sins_node");

    IMUPreintegration ImuP;

    ros::spin();

    return 0;
}

