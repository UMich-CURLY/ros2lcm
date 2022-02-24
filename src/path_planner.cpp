#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <hssh/local_topological/local_topo_graph.h>
#include <utils/serialized_file_io.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace vulcan;
using namespace vulcan::system;
using namespace vulcan::hssh;
using namespace boost;


std::vector<Point<double>> find_path(LTGVertex& start, LTGVertex& goal, LTGraphType& graph){

    int kSearchNodeId = -1;

    LTGraphType::vertex_descriptor start_vertex,goal_vertex;

    LTGraphType::vertex_iterator v, vend, vnext; 
        for (boost::tie(v, vend) = boost::vertices(graph); v != vend; ++v){
            if(graph[*v].gatewayId == start.gatewayId){
                start_vertex = *v;
            }
            if(graph[*v].gatewayId == goal.gatewayId){
                goal_vertex = *v;
            }
        }

    std::vector<double> distances(num_vertices(graph));
    std::vector<LTGraphType::vertex_descriptor> predecessors(num_vertices(graph));

    std::cout<<"Before dijkstra_shortest_paths"<<std::endl;
    dijkstra_shortest_paths(graph, start_vertex,
                            weight_map(get(&LTGEdge::distance, graph))
                            .distance_map(make_iterator_property_map(distances.begin(), get(vertex_index, graph)))
                            .predecessor_map(make_iterator_property_map(predecessors.begin(), get(vertex_index, graph))));
    
    std::cout<<"After dijkstra_shortest_paths"<<std::endl;
    auto nextVertex = goal_vertex;

    std::vector<double> eventDistances;
    std::vector<LocalArea::Id> areaSequence;
    std::vector<Point<double>> eventPointSequence;
    std::vector<int> eventGatewaySequence;

    while(predecessors[nextVertex] != nextVertex)
    {
        auto parentEdge = edge(predecessors[nextVertex], nextVertex, graph);
        assert(parentEdge.second);
        areaSequence.push_back(graph[parentEdge.first].areaId);
        eventDistances.push_back(graph[parentEdge.first].distance);
        eventPointSequence.push_back(graph[nextVertex].position);
        eventGatewaySequence.push_back(graph[nextVertex].gatewayId);

        nextVertex = predecessors[nextVertex];
    }

    eventPointSequence.push_back(graph[start_vertex].position);
    eventGatewaySequence.push_back(kSearchNodeId);

    std::reverse(areaSequence.begin(), areaSequence.end());
    std::reverse(eventDistances.begin(), eventDistances.end());
    std::reverse(eventPointSequence.begin(), eventPointSequence.end());
    std::reverse(eventGatewaySequence.begin(), eventGatewaySequence.end());

    std::cout<<"Path: "<<std::endl;
    for(auto& p:eventPointSequence){
            std::cout<<p<<std::endl;
    }

    std::cout<<"eventDistances: "<<std::endl;
    for(auto& p:eventDistances){
            std::cout<<p<<std::endl;
    }

    std::cout<<"areaSequence: "<<std::endl;
    for(auto& p:areaSequence){
            std::cout<<p<<std::endl;
    }

    std::cout<<"eventGatewaySequence: "<<std::endl;
    for(auto& p:eventGatewaySequence){
            std::cout<<p<<std::endl;
    }

    return eventPointSequence;
}

class path_planner
{
public:
ros::NodeHandle nh_;

ros::Publisher path_pub;

ros::Subscriber path_sub;

path_planner()
{
    ros::NodeHandle private_nh("~");

    path_pub = nh_.advertise<nav_msgs::Path>("/result_path",1000);

    path_sub = nh_.subscribe<nav_msgs::Path>("/robot_1/global_plan", 1, &path_planner::path_callback, this);
}

~path_planner()
{

}

void path_callback(const nav_msgs::Path::ConstPtr& msg);
};



void path_planner::path_callback(const nav_msgs::Path::ConstPtr& msg){
    ROS_INFO("Into path_planner");
    hssh::LocalTopoMap topoMap;

    if(!utils::load_serializable_from_file("/root/catkin_ws/src/ros2lcm/src/Vulcan/build/bin/new_env_laser.ltm", topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << '\n';
        }

    LocalTopoGraph graph(topoMap);
    
    LTGraphType G = graph.getGraph();
    ROS_INFO("GETING POSE");

    Point<double> start_pose(msg->poses.begin()->pose.position.x + 18.713085, msg->poses.begin()->pose.position.y + 61.350861);
    Point<double> goal_pose(msg->poses[4].pose.position.x + 18.713085, msg->poses[4].pose.position.y + 61.350861);

    ROS_INFO("GETING POSE");
    auto start = topoMap.areaContaining(start_pose);
    auto end = topoMap.areaContaining(goal_pose);

    std::pair<Point<double>, int> start_node(start_pose, start->id());
    std::pair<Point<double>, int> goal_node(goal_pose, end->id());

    ROS_INFO("Before find path");
    LocalTopoRoute path = graph.findPath(start_node,goal_node);
    ROS_INFO("After find path");
    nav_msgs::Path path_msg;

    geometry_msgs::PoseStamped path_pose;

    path_pose.pose.position.x = path.front().entryPoint().x - 18.713085;
    path_pose.pose.position.y = path.front().entryPoint().y - 61.350861;

    path_msg.poses.push_back(path_pose);


    for(auto& visit : path)
    {   

        path_pose.pose.position.x = visit.exitPoint().x - 18.713085;
        path_pose.pose.position.y = visit.exitPoint().y - 61.350861;
        path_msg.poses.push_back(path_pose);
    }
    path_msg.header.frame_id = "map";
    ROS_INFO("Published!!!");
    path_pub.publish(path_msg);

}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_pub");

    path_planner path_node;

    ros::spin();
  
    return 0;
}