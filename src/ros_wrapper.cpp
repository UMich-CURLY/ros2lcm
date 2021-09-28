/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     data_queue.cpp
* \author   Collin Johnson
*
* Definition of MetricSLAMDataQueue.
*/

// #include <hssh/metrical/data_queue.h>
// #include <hssh/metrical/data.h>
// #include <laser/line_extraction.h>
// #include <laser/line_extractor_params.h>
#include <core/imu_data.h>
#include <core/odometry.h>
// #include <core/laser_scan.h>
#include <core/motion_state.h>
#include <system/module_communicator.h>
// #include <utils/algorithm_ext.h>
// #include <utils/auto_mutex.h>
#include <utils/timestamp.h>
#include <cassert>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

namespace vulcan
{
namespace system { class ModuleCommunicator; }
namespace hssh
{

class ros_wrapping
{
public:
ros::NodeHandle nh_;
ros::Subscriber sub_odom;
ros::Subscriber sub_imu;
ros::Subscriber sub_scan;

odometry_t* odom_msg;
imu_data_t* imu_msg;
polar_laser_scan_t* scan_msg;
system::ModuleCommunicator* communicator;
int64_t imu_timestamp;                                  //Old imu timestamp in nano seconds 
ros_wrapping()
{
    ros::NodeHandle private_nh("~");
    // if(!lcm.good())
    //     ros::shutdown();
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/odom", 1,  &ros_wrapping::odom_callback, this);
    sub_imu = nh_.subscribe<sensor_msgs::Imu>("/imu", 1,  &ros_wrapping::imu_callback, this);
    sub_scan = nh_.subscribe<sensor_msgs::LaserScan>("/base_scan", 1,  &ros_wrapping::scan_callback, this);
    communicator = new system::ModuleCommunicator();
    imu_timestamp = -1;                               // No imu messahes received yet 
    printf("Timestamp initialized as %ld \n", imu_timestamp);
}

~ros_wrapping()
{

}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void ros_wrapping::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_msg = new odometry_t();
    odom_msg->timestamp = ros::WallTime::now().toNSec()/1000;        ///< Time at which the measurement was made
    odom_msg->id = msg->header.seq;                      ///< Monotonically increasing id so missing odometry can be easily identified
    odom_msg->x = msg->pose.pose.position.x;                   ///< Dead-reckoning x-position of the robot
    odom_msg->y = msg->pose.pose.position.y;                  ///< Dead-reckoning y-position of the robot
    odom_msg->theta = msg->pose.pose.orientation.z;   //quat->z(angle in rad maybe, check?)        ///< Dead-reckoning orientation of the robot
    odom_msg->translation = msg->twist.twist.linear.x;    ///< Distance traveled between last measurement and this measurement
    odom_msg->rotation = msg->twist.twist.angular.z;       ///< Amount of rotation between last measurement and this measurement
    communicator->sendMessage<odometry_t>   (*odom_msg);
}

void ros_wrapping::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    int64_t timestamp = 0;
    printf("Timestamp in callback as %ld \n", imu_timestamp);
    if (imu_timestamp != -1)
    {
      timestamp = ros::WallTime::now().toNSec() - imu_timestamp;
    }
    else
    {
      printf("NULL msg detected");
      timestamp = 0;
    }
    imu_timestamp = ros::WallTime::now().toNSec();
    imu_msg = new imu_data_t();
    imu_msg->timestamp = imu_timestamp/1000;             ///< Time at which the IMU data was read (microseconds)
    imu_msg->sequenceNumber = msg->header.seq;       ///< Monotonically increasing number that identifies the particular measurement
    imu_msg->timeDelta = 0;              ///< Time difference between this reading and last reading in IMU data sequence (microseconds)
    imu_msg->acceleration[0] = msg->linear_acceleration.x;          ///< Measured acceleration values (x, y, z)
    imu_msg->acceleration[1] = msg->linear_acceleration.y;          ///< Measured acceleration values (x, y, z)
    imu_msg->acceleration[2] = msg->linear_acceleration.z;          ///< Measured acceleration values (x, y, z)
    imu_msg->rotationalVelocity[0] = msg->angular_velocity.z;    ///< Measured rotational velocities (deltaYaw, deltaPitch, deltaRoll)
    imu_msg->rotationalVelocity[1] = msg->angular_velocity.y;    ///< Measured rotational velocities (deltaYaw, deltaPitch, deltaRoll)
    imu_msg->rotationalVelocity[2] = msg->angular_velocity.x;    ///< Measured rotational velocities (deltaYaw, deltaPitch, deltaRoll)
    imu_msg->orientation[0] = msg->orientation.z;           ///< Estimate of the global orientation of the robot (yaw, pitch, roll)
    imu_msg->orientation[1] = msg->orientation.y;           ///< Estimate of the global orientation of the robot (yaw, pitch, roll)
    imu_msg->orientation[2] = msg->orientation.x;           ///< Estimate of the global orientation of the robot (yaw, pitch, roll)

    imu_msg->gravityMagnitude = 9.8;
    communicator->sendMessage<imu_data_t>   (*imu_msg);
}

void ros_wrapping::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg = new polar_laser_scan_t();
    scan_msg->laserId = 0;                    ///< Unique id assigned to the source laser --- guaranteed to start at 0 and be sequential for each new laser added

    scan_msg->timestamp = ros::WallTime::now().toNSec()/1000;                  ///< Time at which the measurement was taken
    scan_msg->scanId = msg->header.seq;                     ///< Monotonically increasing ID for the scans to allow multiple modules to sync on laser scans

    scan_msg->startAngle = msg->angle_min;                   ///< Angle the first range in the scan points, in laser coordinates
    scan_msg->angularResolution = msg->angle_increment;            ///< Change in angle between each measurement. - = CCW scan, + = CW scan

    scan_msg->numRanges = msg->ranges.size();                ///< Number of range values in the scan
    scan_msg->ranges = msg->ranges;                   ///< Measured ranges -- negative values should be ignored as bad readings
    for( int i = 0; i<scan_msg->numRanges; i++)
    {
      scan_msg->intensities.push_back((unsigned int)msg->intensities[i]);
    }                                                 ///< Intensity values for each of the ranges

    // Parameters of the particular rangefinder
    scan_msg->maxRange = msg->range_max;             ///< Maximum range that can be measured by the laser
    scan_msg->scanPeriod = msg->scan_time;           ///< Seconds per rotation of 2pi radians. Equivalent to 60/rpm of laser
    pose_6dof_t offset = pose_6dof_t();               ///< Offset of the rangefinder from the center of the robot coordinate frame
    scan_msg->offset = offset;
    communicator->sendMessage<polar_laser_scan_t>   (*scan_msg, "SENSOR_LASER_FRONT_6DOF");
}

}
}

int main(int argc, char** argv)
{
  using namespace vulcan;
  using namespace vulcan::system;
  using namespace vulcan::hssh;
  ros::init(argc, argv, "lcm_republisher");
  
  ros_wrapping roswrapper;
  ros::spin();
  
  return 0;
}
