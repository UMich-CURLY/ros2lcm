#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <hssh/local_topological/local_topo_graph.h>
#include <utils/serialized_file_io.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <nav_msgs/GetPlan.h>

using namespace vulcan;
using namespace vulcan::system;
using namespace vulcan::hssh;
using namespace boost;

bool path_callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res){

    ROS_INFO("Into path_planner");
    hssh::LocalTopoMap topoMap;

    if(!utils::load_serializable_from_file("/root/catkin_ws/src/ros2lcm/src/Vulcan/build/bin/new_env_laser.ltm", topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
        }

    LocalTopoGraph graph(topoMap);
    
    LTGraphType G = graph.getGraph();
    ROS_INFO("GETING POSE");

    Point<double> start_pose(req.start.pose.position.x + 18.713085, req.start.pose.position.y + 61.350861);
    Point<double> goal_pose(req.goal.pose.position.x + 18.713085, req.goal.pose.position.y + 61.350861);

    ROS_INFO("GETING POSE");
    auto start = topoMap.areaContaining(start_pose);
    auto end = topoMap.areaContaining(goal_pose);

    std::pair<Point<double>, int> start_node(start_pose, start->id());
    std::pair<Point<double>, int> goal_node(goal_pose, end->id());

    ROS_INFO("Before find path");
    LocalTopoRoute path = graph.findPath(start_node,goal_node);
    ROS_INFO("After find path");
    // nav_msgs::Path path_msg;

    geometry_msgs::PoseStamped path_pose;

    path_pose.pose.position.x = path.front().entryPoint().x - 18.713085;
    path_pose.pose.position.y = path.front().entryPoint().y - 61.350861;

    res.plan.poses.push_back(path_pose);


    for(auto& visit : path)
    {   

        path_pose.pose.position.x = visit.exitPoint().x - 18.713085;
        path_pose.pose.position.y = visit.exitPoint().y - 61.350861;
        res.plan.poses.push_back(path_pose);
    }
    res.plan.header.frame_id = "map";

    return true;

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_pub");

    ros::NodeHandle nh_;
    
    ros::ServiceServer service = nh_.advertiseService("plan_path", path_callback);

    ros::spin();
  
    return 0;
}