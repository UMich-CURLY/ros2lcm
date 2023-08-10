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
#include <yaml-cpp/yaml.h>

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
double x_offset;
double y_offset;
path_planner(const LocalTopoMap& map): topoMap(map), graph(LocalTopoGraph(map))
{
    G = graph.getGraph();
    YAML::Node config;
    config = YAML::LoadFile("/root/catkin_ws/src/ros2lcm/trying_VV.yaml");
    if (config.IsNull())
    {
        std::cout << "Error: YAML::Node is Null, indicating the yaml file was empty or invalid.\n";
    }
    YAML::Node origin = config["origin"];
    x_offset = origin[0].as<double>();
    y_offset = origin[1].as<double>();
    if (config["origin"])
    {
        std::cout << "Yes";
        
    }

}

~path_planner()
{

}


};

bool path_planner::path_callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res){
    std::cout << "Okay for now"<<std::endl;
    double delta_x = req.start.pose.position.x - req.goal.pose.position.x;
    double delta_y = req.start.pose.position.y - req.goal.pose.position.y;
    tf2::Quaternion myQuaternion;
    geometry_msgs::PoseStamped goal;

    if(sqrt(pow(delta_x,2) + pow(delta_y,2)) > 0.1){

    Point<double> start_pose(req.start.pose.position.x -this->x_offset, req.start.pose.position.y -this->y_offset);
    Point<double> goal_pose(req.goal.pose.position.x -this->x_offset, req.goal.pose.position.y -this->y_offset);
    // for(std::size_t n = 0; n < topoMap.areas_.size(); ++n)
    // {
    //     std::cout << topoMap.areas_[n].extent << std::endl;
    // }
    
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


    if(path.cells.size() != 0){

    for(int i=0;i < path.cells.size() - 1; i++){
        auto global_point_current = utils::grid_point_to_global_point(path.cells[i],topoMap.voronoiSkeleton());
        auto global_point_next = utils::grid_point_to_global_point(path.cells[i + 1], topoMap.voronoiSkeleton());
        geometry_msgs::PoseStamped path_pose;
        path_pose.pose.position.x = global_point_current.x + this->x_offset;
        path_pose.pose.position.y = global_point_current.y + this->y_offset;

        double delta_y = global_point_next.y - global_point_current.y;
        double delta_x = global_point_next.x - global_point_current.x;

        double angle = atan2(delta_y,delta_x);

        myQuaternion.setRPY( 0, 0, angle );

        myQuaternion.normalize();

        path_pose.pose.orientation = tf2::toMsg(myQuaternion);

        path_pose.header.frame_id = "map";
        res.plan.poses.push_back(path_pose);
        // std::cout<<"Path pose: "<<path_pose.pose.position.x<<","<<path_pose.pose.position.y<<std::endl;
        }
    }
}

    else{
        double angle = atan2(delta_y,delta_x);
        myQuaternion.setRPY( 0, 0, angle );

        myQuaternion.normalize();
        goal.pose.position.x = req.goal.pose.position.x;
        goal.pose.position.y = req.goal.pose.position.y;

        // goal.pose.orientation = tf2::toMsg(myQuaternion);
        goal.pose.orientation = req.goal.pose.orientation;

        goal.header.frame_id = "map";
        res.plan.poses.push_back(goal);
        res.plan.header.frame_id = "map";

        return true;

    }
    goal.pose.position.x = req.goal.pose.position.x;
    goal.pose.position.y = req.goal.pose.position.y;

    // goal.pose.orientation = tf2::toMsg(myQuaternion);
    goal.pose.orientation = req.goal.pose.orientation;
    goal.header.frame_id = "map";
    res.plan.poses.push_back(goal);
    res.plan.header.frame_id = "map";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_pub");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    hssh::LocalTopoMap topoMap;
    std::cout<<"In main !!"<<std::endl;
    if(!utils::load_serializable_from_file("/root/catkin_ws/src/ros2lcm/trying_VV.ltm", topoMap))
        {
            std::cout << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
        } 
    path_planner planner(topoMap);

    // Point<double> start_pose(6.66, 3.88);
    // Point<double> goal_pose(18.91, 61.05);

    // auto start = topoMap.areaContaining(start_pose);
    // auto end = topoMap.areaContaining(goal_pose);

    // std::pair<Point<double>, int> start_node(start_pose, start->id());
    // std::pair<Point<double>, int> goal_node(goal_pose, end->id());


    // auto path = find_path_along_skeleton(utils::global_point_to_grid_cell_round(start_node.first, topoMap.voronoiSkeleton()),
    //                                      utils::global_point_to_grid_cell_round(goal_node.first, topoMap.voronoiSkeleton()),
    //                                      SKELETON_CELL_REDUCED_SKELETON,
    //                                      topoMap.voronoiSkeleton());
    // // std::vector<Point<double>> global_path;
    // tf2::Quaternion myQuaternion;
    // double distance_sum = 0;
    // for(int i=0;i < path.cells.size() - 1; i++){

    //     auto global_point_current = utils::grid_point_to_global_point(path.cells[i],topoMap.voronoiSkeleton());
    //     auto global_point_next = utils::grid_point_to_global_point(path.cells[i + 1], topoMap.voronoiSkeleton());
    //     geometry_msgs::PoseStamped path_pose;
    //     path_pose.pose.position.x = global_point_current.x - 18.713085;
    //     path_pose.pose.position.y = global_point_current.y - 61.350861;

    //     double delta_y = global_point_next.y - global_point_current.y;
    //     double delta_x = global_point_next.x - global_point_current.x;

    //     double angle = atan2(delta_y,delta_x);
    //     distance_sum += sqrt(pow(delta_x,2) + pow(delta_y,2));

    //     myQuaternion.setRPY( 0, 0, angle );

    //     myQuaternion.normalize();

    //     path_pose.pose.orientation = tf2::toMsg(myQuaternion);

    //     path_pose.header.frame_id = "map";
    //     // res.plan.poses.push_back(path_pose);
    //     // std::cout<<global_point<<std::endl;
    //     std::cout<<path_pose.pose.orientation<<std::endl;
    // }


    // std::cout<<"Path.length(): "<<path.length<<std::endl;
    // std::cout<<"distance_sum: "<<distance_sum<<std::endl
    ros::ServiceServer service = nh_.advertiseService("/plan_path", &path_planner::path_callback, &planner);
    ros::spin();
  
    return 0;
}