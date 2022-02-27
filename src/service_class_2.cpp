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

class path_planner
{
public:

hssh::LocalTopoMap topoMap;
LocalTopoGraph graph;
LTGraphType G;
ros::ServiceClient client;

bool path_callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);

path_planner(const LocalTopoMap& map, ros::ServiceClient& client_): topoMap(map), graph(LocalTopoGraph(map)), client(client_)
{
    ROS_INFO("Into path_planner");
    G = graph.getGraph();

    boost::property_map<LTGraphType, Point<float> LTGVertex::*>::type position = get(&LTGVertex::position, G);
    boost::property_map<LTGraphType, float LTGEdge::*>::type distance = get(&LTGEdge::distance, G);

    boost::graph_traits<LTGraphType>::vertex_iterator i, end;
    boost::graph_traits<LTGraphType>::out_edge_iterator ei, edge_end;

    boost::graph_traits<LTGraphType>::edge_iterator e_it, e_end;
    for(std::tie(e_it, e_end) = boost::edges(G); e_it != e_end; ++e_it)
    {   
        auto start_position = position[boost::source(*e_it, G)];
        auto goal_position = position[boost::target(*e_it, G)];

        start_position.x -= 18.713085;
        goal_position.x -= 18.713085;
        start_position.y -= 61.350861;
        goal_position.y -= 61.350861;
        
        geometry_msgs::PoseStamped start_msg;
        geometry_msgs::PoseStamped goal_msg;

        start_msg.header.frame_id = "map";
        start_msg.header.stamp = ros::Time();

        start_msg.pose.position.x = start_position.x;
        start_msg.pose.position.y = start_position.y;

        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time();

        goal_msg.pose.position.x = goal_position.x;
        goal_msg.pose.position.y = goal_position.y;

        nav_msgs::GetPlan srv;
        
        // if(client.call(srv)){
        //     ROS_INFO("Called the A* service!!!!");
        // }

        srv.request.start = start_msg;
        srv.request.goal = goal_msg;
        srv.request.tolerance = 0.1;

        std::cout<< "Topic of service: "<<client.getService()<<std::endl;
        std::cout<< "Active? "<<client.exists()<<std::endl;
        if(client.call(srv.request,srv.response)){
            ROS_INFO("Called the A* service!!!!");
        }

        int sum_ = 0;
        for(int i = 1; i < srv.response.plan.poses.size(); i++){
            sum_ += sqrt(pow((srv.response.plan.poses[i].pose.position.x - srv.response.plan.poses[i-1].pose.position.x),2) + pow((srv.response.plan.poses[i].pose.position.y - srv.response.plan.poses[i-1].pose.position.y), 2));
        }

        // ROS_INFO("Changing distance");
        distance[*e_it] = 1.0;

        std::cout<<start_position << "-" << distance[*e_it]<<"-"<<goal_position<<std::endl;

    }

    ROS_INFO("END OF CALLING MOVE_BASE TO UPDATE");

    // graph.setGraph(G);
    graph.getskeletonGraph();
}


// path_planner(const LocalTopoMap& map, ros::ServiceClient& client_): topoMap(map), client(client_)
// {
//     ROS_INFO("Into path_planner");
//      std::unordered_map<int, LTGraphType::vertex_descriptor> gatewayToVertex;

//     // For each area, create vertices for the gateways and add edges between each pair of gateways
//     for(auto& area : topoMap)
//     {
//         std::vector<Gateway> areaGateways = area->gateways();

//         // Add all gateways
//         for(const auto& g : areaGateways)
//         {
//             // Validate that we haven't loaded some degenerate gateway
//             auto path = path_to_skeleton(g.skeletonCell(), SKELETON_CELL_REDUCED_SKELETON, topoMap.voronoiSkeleton());
//             print(path)
//             // Ignore any gateways for which a proper path to the skeleton can't be found.
//             if(path.result != VoronoiPathResult::success)
//             {
//                 continue;
//             }

//             if(gatewayToVertex.find(g.id()) == gatewayToVertex.end())
//             {
//                 LTGVertex vertex;
//                 vertex.position = g.center();
//                 vertex.gatewayId = g.id();
//                 gatewayToVertex[g.id()] = boost::add_vertex(vertex, graph_);
//                 gateways_[g.id()] = g;
//             }

//             areaVertices_[area->id()].push_back(gatewayToVertex[g.id()]);
//         }

//         // Add an edge between all pairs of gateways
//         for(std::size_t n = 0; n < areaGateways.size(); ++n)
//         {
//             for(std::size_t i = n + 1; i < areaGateways.size(); ++i)
//             {
//                 LTGEdge edge;
//                 edge.areaId = area->id();

//                 // Find the path between the gateways
//                 auto path = find_path_along_skeleton(areaGateways[n].skeletonCell(),
//                                                      areaGateways[i].skeletonCell(),
//                                                      SKELETON_CELL_REDUCED_SKELETON,
//                                                      topoMap.voronoiSkeleton());

//                 if(path.result == VoronoiPathResult::success)
//                 {
//                     edge.distance = path.length;
//                     boost::add_edge(gatewayToVertex.at(areaGateways[n].id()),
//                                     gatewayToVertex.at(areaGateways[i].id()),
//                                     edge,
//                                     graph_);
//                 }
//                 else
//                 {
//                     std::cerr << "ERROR: LocalTopoGraph: Failed to find path between gateways: "
//                         << areaGateways[n] << " to " << areaGateways[i] << " in " << area->id() << '\n';
//                 }
//             }
//         }
//     }
// }
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
    // graph.getskeletonGraph();
    LocalTopoRoute path = graph.findPath(start_node,goal_node);

    geometry_msgs::PoseStamped path_pose;

    path_pose.pose.position.x = path.length();
    path_pose.pose.position.y = 0;
    path_pose.header.frame_id = "map";
    res.plan.poses.push_back(path_pose);

    ROS_INFO("Publishing distances");
    res.plan.header.frame_id = "map";
    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_pub_1");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    hssh::LocalTopoMap topoMap;
    if(!utils::load_serializable_from_file("/root/catkin_ws/src/ros2lcm/src/Vulcan/build/bin/new_env_laser.ltm", topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
        } 

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    path_planner planner(topoMap, client);
    ros::ServiceServer service = nh_.advertiseService("/get_distance", &path_planner::path_callback, &planner);

    ros::spin();
  
    return 0;
}