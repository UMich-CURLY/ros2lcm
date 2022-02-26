#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <hssh/local_topological/local_topo_graph.h>
#include <utils/serialized_file_io.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <nav_msgs/GetPlan.h>
#include <hssh/local_topological/area.h>
#include <hssh/local_topological/area_detection/voronoi/search.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
using namespace vulcan;
using namespace vulcan::system;
using namespace vulcan::hssh;
using namespace boost;

class path_planner
{
public:
hssh::LocalTopoMap topoMap;
LocalTopoGraph graph;
LTGraphType G;
bool path_callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);

path_planner(const LocalTopoMap& map): topoMap(map), graph(LocalTopoGraph(map))
{
    G = graph.getGraph();
}

~path_planner()
{

}


};


bool path_planner::path_callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res){
    Point<double> start_pose(req.start.pose.position.x + 18.713085, req.start.pose.position.y + 61.350861);
    Point<double> goal_pose(req.goal.pose.position.x + 18.713085, req.goal.pose.position.y + 61.350861);

    auto start = topoMap.areaContaining(start_pose);
    auto end = topoMap.areaContaining(goal_pose);

    std::pair<Point<double>, int> start_node(start_pose, start->id());
    std::pair<Point<double>, int> goal_node(goal_pose, end->id());

    LocalTopoRoute path = graph.findPath(start_node,goal_node);

    geometry_msgs::PoseStamped path_pose;

    path_pose.pose.position.x = path.front().entryPoint().x - 18.713085;
    path_pose.pose.position.y = path.front().entryPoint().y - 61.350861;
    path_pose.header.frame_id = "map";
    res.plan.poses.push_back(path_pose);
    for(auto& visit : path)
    {   
        path_pose.pose.position.x = visit.exitPoint().x - 18.713085;
        path_pose.pose.position.y = visit.exitPoint().y - 61.350861;
        res.plan.poses.push_back(path_pose);
    }
    ROS_INFO("Publishing distances");
    res.plan.header.frame_id = "map";
    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_pub");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    hssh::LocalTopoMap topoMap;
    if(!utils::load_serializable_from_file("/mnt/Sauna/new_env_laser.ltm", topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
        } 
    path_planner planner(topoMap);
    ros::ServiceServer service = nh_.advertiseService("/plan_path", &path_planner::path_callback, &planner);
    ros::spin();
  
    return 0;
}