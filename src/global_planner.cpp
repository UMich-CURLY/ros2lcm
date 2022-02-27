#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

 //register this planner as a BaseGlobalPlanner plugin

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

// using namespace std;
// using namespace vulcan;
// using namespace vulcan::system;
// using namespace vulcan::hssh;
// using namespace boost;

 //Default Constructor
namespace global_planner{

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
  //     if(!utils::load_serializable_from_file("/root/catkin_ws/src/ros2lcm/src/Vulcan/build/bin/new_env_laser.ltm", topoMap))
  //       {
  //           std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
  //       } 


      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      
      initialized_ = true;
    }
    // else
      ROS_WARN("This planner has already been initialized... doing nothing");
  
 }


// double GlobalPlanner::getYawFromQuat(geometry_msgs::Quaternion quat)
//     {
//         tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
//         tf::Matrix3x3 m(q);
//         double roll, pitch, yaw;
//         m.getRPY(roll, pitch, yaw);

//         return yaw;
//     }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    // LocalTopoGraph graph(topoMap);
    // LTGraphType G = graph.getGraph();

    // Point<double> start_pose(start_.pose.position.x + 18.713085, start_.pose.position.y + 61.350861);
    // Point<double> goal_pose(goal.pose.position.x + 18.713085, goal.pose.position.y + 61.350861);

    // auto start = topoMap.areaContaining(start_pose);
    // auto end = topoMap.areaContaining(goal_pose);

    // std::pair<Point<double>, int> start_node(start_pose, start->id());
    // std::pair<Point<double>, int> goal_node(goal_pose, end->id());

    // auto path = find_path_along_skeleton(utils::global_point_to_grid_cell_round(start_node.first, topoMap.voronoiSkeleton()),
    //                                      utils::global_point_to_grid_cell_round(goal_node.first, topoMap.voronoiSkeleton()),
    //                                      SKELETON_CELL_REDUCED_SKELETON,
    //                                      topoMap.voronoiSkeleton());
    // // std::vector<Point<double>> global_path;
    // // for(auto& cell : path.cells){
    // tf2::Quaternion myQuaternion;
    // for(int i=0;i < path.cells.size() - 1; i++){

    //     auto global_point_current = utils::grid_point_to_global_point(path.cells[i],topoMap.voronoiSkeleton());
    //     auto global_point_next = utils::grid_point_to_global_point(path.cells[i + 1], topoMap.voronoiSkeleton());
    //     geometry_msgs::PoseStamped path_pose;
    //     path_pose.pose.position.x = global_point_current.x - 18.713085;
    //     path_pose.pose.position.y = global_point_current.y - 61.350861;

    //     double delta_y = global_point_next.y - global_point_current.y;
    //     double delta_x = global_point_next.x - global_point_current.x;

    //     double angle = atan2(delta_y,delta_x);

    //     myQuaternion.setRPY( 0, 0, angle );

    //     myQuaternion.normalize();

    //     path_pose.pose.orientation = tf2::toMsg(myQuaternion);

    //     path_pose.header.frame_id = "map";
    //     plan.push_back(path_pose);
    //     // std::cout<<global_point<<std::endl;
    // }
    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/plan_path");

    nav_msgs::GetPlan srv;
    srv.request.start = start_;
    srv.request.goal = goal;
    srv.request.tolerance = 0.1;

    if(client.call(srv)){
      ROS_INFO("Called our Service!!!");
    }


    plan = srv.response.plan.poses;

    for(auto& p:plan){
      std::cout<<"pose: "<<p.pose.position.x<<","<<p.pose.position.y<<std::endl;
    }

    return true;

 }
};