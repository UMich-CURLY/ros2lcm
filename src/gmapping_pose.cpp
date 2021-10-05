#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <core/odometry.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

class gmapping_pose
{
  public:
  ros::NodeHandle n;
  ros::Subscriber sub_odom = n.subscribe<nav_msgs::Odometry>("/odom", 1, &gmapping_pose::odom_callback, this);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("gmapping/pose", 1000);
  ros::Subscriber sub_gt = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &gmapping_pose::gt_callback, this);
  ros::Publisher gt_pub = n.advertise<geometry_msgs::PoseStamped>("gazebo/pose", 1000);
  tf::StampedTransform transform;
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void gt_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
  int seq;
};
void gmapping_pose::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    tf::Vector3 raw_position, transformed_position;
    // raw_position = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    raw_position = tf::Vector3(0.0,0.0,0.0);
    transformed_position = transform*raw_position;
    pose.pose.position.x = transformed_position.getX();                   ///< Dead-reckoning x-position of the robot
    pose.pose.position.y = transformed_position.getY();                  ///< Dead-reckoning y-position of the robot
    pose.pose.position.z = transformed_position.getZ();

    tf::Quaternion raw_quat, transformed_quat;
    // raw_quat = tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    raw_quat = tf::Quaternion(0.0,0.0,0.0,1.0);
    transformed_quat = transform*raw_quat;
    pose.pose.orientation.x = transformed_quat.getX();
    pose.pose.orientation.y = transformed_quat.getY();
    pose.pose.orientation.z = transformed_quat.getZ();
    pose.pose.orientation.w = transformed_quat.getW();

    pose.header.stamp = ros::Time::now();
    pose.header.seq = msg->header.seq;
    seq = msg->header.seq;
    pose.header.frame_id = "map";
    pose_pub.publish(pose);
}

void gmapping_pose::gt_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.seq = seq;
    pose.header.frame_id = "map";
    pose.pose = msg->pose[5];
    gt_pub.publish(pose);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener"); 
  gmapping_pose object;
  ros::Rate rate(10.0);
  tf::TransformListener listener;
  while (object.n.ok()){
    
    try{
      listener.lookupTransform("/map", "base_link",  
                               ros::Time(0), object.transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};