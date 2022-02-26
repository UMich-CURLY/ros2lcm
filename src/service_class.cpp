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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    tf2::Quaternion myQuaternion;
    for(int i=0;i < path.cells.size() - 1; i++){

        auto global_point_current = utils::grid_point_to_global_point(path.cells[i],topoMap.voronoiSkeleton());
        auto global_point_next = utils::grid_point_to_global_point(path.cells[i + 1], topoMap.voronoiSkeleton());
        geometry_msgs::PoseStamped path_pose;
        path_pose.pose.position.x = global_point_current.x - 18.713085;
        path_pose.pose.position.y = global_point_current.y - 61.350861;

        double delta_y = global_point_next.y - global_point_current.y;
        double delta_x = global_point_next.x - global_point_current.x;

        double angle = atan2(delta_y,delta_x);

        myQuaternion.setRPY( 0, 0, angle );

        myQuaternion.normalize();

        path_pose.pose.orientation = tf2::toMsg(myQuaternion);

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
    // std::vector<Point<double>> global_path;
    tf2::Quaternion myQuaternion;
    for(int i=0;i < path.cells.size() - 1; i++){

        auto global_point_current = utils::grid_point_to_global_point(path.cells[i],topoMap.voronoiSkeleton());
        auto global_point_next = utils::grid_point_to_global_point(path.cells[i + 1], topoMap.voronoiSkeleton());
        geometry_msgs::PoseStamped path_pose;
        path_pose.pose.position.x = global_point_current.x - 18.713085;
        path_pose.pose.position.y = global_point_current.y - 61.350861;

        double delta_y = global_point_next.y - global_point_current.y;
        double delta_x = global_point_next.x - global_point_current.x;

        double angle = atan2(delta_y,delta_x);

        myQuaternion.setRPY( 0, 0, angle );

        myQuaternion.normalize();

        path_pose.pose.orientation = tf2::toMsg(myQuaternion);

        path_pose.header.frame_id = "map";
        // res.plan.poses.push_back(path_pose);
        // std::cout<<global_point<<std::endl;
        std::cout<<path_pose.pose.orientation<<std::endl;
    }
    ros::ServiceServer service = nh_.advertiseService("/plan_path", &path_planner::path_callback, &planner);
    ros::spin();
  
    return 0;
}