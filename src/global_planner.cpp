#include <pluginlib/class_list_macros.h>
#include "global_planner.h"





 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner_tribhi::GlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;
using namespace vulcan;
using namespace vulcan::system;
using namespace vulcan::hssh;
using namespace boost;

 //Default Constructor
namespace global_planner_tribhi{

  struct points_for_spline{
      double x;
      double y;
      double heading;
    };

    // bool sort_function(points_for_spline a, points_for_spline b)
    // {
    //   return a.x<b.x;
    // }

 GlobalPlanner::GlobalPlanner (): costmap_ros_(NULL), initialized_(false){
  ROS_INFO("Yeah 1 ");
 }

 GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros): costmap_ros_(NULL), initialized_(false){
   ROS_INFO("Yeah 2");
   initialize(name, costmap_ros);

 }


 void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

  if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  
 }


double GlobalPlanner::getYawFromQuat(geometry_msgs::Quaternion quat)
    {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

   //  points_for_spline temp_pt, start_pt, goal_pt;
   //  std::vector<points_for_spline> points;
   //  double path_length, extra_pt_dist;
   //  start_pt.x = start.pose.position.x;
   //  start_pt.y = start.pose.position.y;
   //  start_pt.heading = getYawFromQuat(start.pose.orientation);
   //  goal_pt.heading = getYawFromQuat(goal.pose.orientation);
   //  goal_pt.x = goal.pose.position.x;
   //  goal_pt.y = goal.pose.position.y;
   //  path_length = sqrt(pow(start_pt.x-goal_pt.x,2));//+pow(start_pt.y-goal_pt.y,2));
   //  extra_pt_dist = path_length/10;
   //  temp_pt.x = start_pt.x-2*extra_pt_dist*cos(start_pt.heading);
   //  temp_pt.y = start_pt.y-2*extra_pt_dist*sin(start_pt.heading);
   //  points.push_back(temp_pt);
   //  temp_pt.x = start_pt.x-extra_pt_dist*cos(start_pt.heading);
   //  temp_pt.y = start_pt.y-extra_pt_dist*sin(start_pt.heading);
   //  points.push_back(temp_pt);
   //  points.push_back(start_pt);
   //  points.push_back(goal_pt);
   //  temp_pt.x = goal_pt.x+extra_pt_dist*cos(goal_pt.heading);
   //  temp_pt.y = goal_pt.y+extra_pt_dist*sin(goal_pt.heading);
   //  points.push_back(temp_pt);
   //  temp_pt.x = goal_pt.x+2*extra_pt_dist*cos(goal_pt.heading);
   //  temp_pt.y = goal_pt.y+2*extra_pt_dist*sin(goal_pt.heading);
   //  points.push_back(temp_pt);
   //  std::vector<points_for_spline> path_points;

   //  std::vector<double> x(6),y(6);
   //    std::sort(points.begin(), points.end(), sort_function);
   //    for (unsigned int i =0; i<points.size(); i++)
   //    {
   //      x.at(i) = points.at(i).x;
   //      y.at(i) = points.at(i).y;
   //    }
      
   //    tk::spline s;
   //    s.set_points(x,y);
   //  if(goal_pt.x > start_pt.x)
   //  {
   //    for (double i = start_pt.x; i<goal_pt.x; i+=extra_pt_dist)
   //    {
   //        temp_pt.x = i;
   //        temp_pt.y = s(i);
   //        double next_y = s(i+extra_pt_dist);
   //        temp_pt.heading = atan2(next_y-temp_pt.y, extra_pt_dist);
   //        path_points.push_back(temp_pt);
   //    }
   //  }
   //  else // Do something for moving in just y 
   //  {
   //    for (double i = start_pt.x; i<goal_pt.x; i-=extra_pt_dist)
   //    {
   //        temp_pt.x = i;
   //        temp_pt.y = s(i);
   //        double next_y = s(i-extra_pt_dist);
   //        temp_pt.heading = atan2(next_y-temp_pt.y, -extra_pt_dist);
   //        path_points.push_back(temp_pt);
   //    }
   //  }
    
   //  cout << "plan length is " << path_points.size() << endl;
   //  for (int i=0; i<path_points.size(); i++){
   //   geometry_msgs::PoseStamped new_goal = goal;
   //   tf::Quaternion goal_quat = tf::createQuaternionFromYaw(path_points.at(i).heading);

   //    new_goal.pose.position.x = path_points.at(i).x;
   //    new_goal.pose.position.y = path_points.at(i).y;
   //    new_goal.pose.orientation.x = goal_quat.x();
   //    new_goal.pose.orientation.y = goal_quat.y();
   //    new_goal.pose.orientation.z = goal_quat.z();
   //    new_goal.pose.orientation.w = goal_quat.w();

   // plan.push_back(new_goal);
   // }
   // plan.push_back(goal);

  // for (int i=0; i<2; i++){
  //    geometry_msgs::PoseStamped new_goal = goal;
  //    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

  //     new_goal.pose.position.x = 0.0;
  //     new_goal.pose.position.y = 0.0;
  //     new_goal.pose.orientation.x = goal_quat.x();
  //     new_goal.pose.orientation.y = goal_quat.y();
  //     new_goal.pose.orientation.z = goal_quat.z();
  //     new_goal.pose.orientation.w = goal_quat.w();

  //  plan.push_back(new_goal);
  //  }
  //  plan.push_back(goal);
  return true;
 }
};