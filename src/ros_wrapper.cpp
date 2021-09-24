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

odometry_t* odom_msg;
imu_data_t* imu_msg;
system::ModuleCommunicator* communicator;
ros_wrapping()
{
    ros::NodeHandle private_nh("~");
    // if(!lcm.good())
    //     ros::shutdown();
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/odom", 1,  &ros_wrapping::odom_callback, this);
    sub_imu = nh_.subscribe<sensor_msgs::Imu>("/imu", 1,  &ros_wrapping::imu_callback, this);
    communicator = new system::ModuleCommunicator();
    
}

~ros_wrapping()
{

}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
};

void ros_wrapping::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_t* odom_msg;
    odom_msg = new odometry_t();
    odom_msg->timestamp = msg->header.seq;
    odom_msg->x = msg->pose.pose.position.x;
    odom_msg->y = msg->pose.pose.position.y;
    odom_msg->theta = msg->pose.pose.orientation.z;
    odom_msg->translation = msg->twist.twist.linear.x;
    odom_msg->rotation = msg->twist.twist.angular.z;
    communicator->sendMessage<odometry_t>   (*odom_msg);
}

void ros_wrapping::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg = new imu_data_t();
    imu_msg->timestamp = msg->header.stamp.secs;              ///< Time at which the IMU data was read (microseconds)

    imu_msg->sequenceNumber = msg->header.seq;       ///< Monotonically increasing number that identifies the particular measurement
    imu_msg->timeDelta = msg->header.nsecs;              ///< Time difference between this reading and last reading in IMU data sequence (microseconds)

    imu_msg->acceleration[3] = msg->linear_acceleration;          ///< Measured acceleration values (x, y, z)
    imu_msg->rotationalVelocity[3] = msg->angular_velocity;    ///< Measured rotational velocities (deltaYaw, deltaPitch, deltaRoll)
    imu_msg->orientation[3] = msg->orientation;           ///< Estimate of the global orientation of the robot (yaw, pitch, roll)

    imu_msg->gravityMagnitude = 9.8;
    communicator->sendMessage<imu_data_t>   (*imu_msg);
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
