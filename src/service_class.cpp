#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <hssh/local_topological/local_topo_graph.h>
#include <utils/serialized_file_io.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <nav_msgs/GetPlan.h>
#include <hssh/local_topological/area_detection/voronoi/search.h>

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

    auto path = find_path_along_skeleton(utils::global_point_to_grid_cell_round(start_node.first, topoMap.voronoiSkeleton()),
                                         utils::global_point_to_grid_cell_round(goal_node.first, topoMap.voronoiSkeleton()),
                                         SKELETON_CELL_REDUCED_SKELETON,
                                         topoMap.voronoiSkeleton());
    // std::vector<Point<double>> global_path;
    // for(auto& cell : path.cells){
    for(int i=0;i < path.cells.size(); i++){
        auto global_point = utils::grid_point_to_global_point(path.cells[i],topoMap.voronoiSkeleton());
        geometry_msgs::PoseStamped path_pose;
        path_pose.pose.position.x = global_point.x - 18.713085;
        path_pose.pose.position.y = global_point.y - 61.350861;

        path_pose.header.frame_id = "map";
        res.plan.poses.push_back(path_pose);
        // std::cout<<global_point<<std::endl;
    }

    res.plan.header.frame_id = "map";
    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_pub");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    hssh::LocalTopoMap topoMap;

    if(!utils::load_serializable_from_file("/root/catkin_ws/src/ros2lcm/src/Vulcan/build/bin/new_env_laser.ltm", topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
        } 
    path_planner planner(topoMap);

    Point<double> start_pose(6.66, 3.88);
    Point<double> goal_pose(18.91, 61.05);

    auto start = topoMap.areaContaining(start_pose);
    auto end = topoMap.areaContaining(goal_pose);

    std::pair<Point<double>, int> start_node(start_pose, start->id());
    std::pair<Point<double>, int> goal_node(goal_pose, end->id());


    auto path = find_path_along_skeleton(utils::global_point_to_grid_cell_round(start_node.first, topoMap.voronoiSkeleton()),
                                         utils::global_point_to_grid_cell_round(goal_node.first, topoMap.voronoiSkeleton()),
                                         SKELETON_CELL_REDUCED_SKELETON,
                                         topoMap.voronoiSkeleton());
    std::vector<Point<double>> global_path;
    for(auto& cell : path.cells){
        auto global_point = utils::grid_point_to_global_point(cell,topoMap.voronoiSkeleton());
        global_path.push_back(global_point);
        std::cout<<global_point<<std::endl;
    }

    ros::ServiceServer service = nh_.advertiseService("/plan_path", &path_planner::path_callback, &planner);
    ros::spin();
  
    return 0;
}