#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <hssh/local_topological/local_topo_graph.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>


// void findPath(Point<float> start, Point<float> end)
// {

// }

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_pub");


 std::ifstream classFile("/root/catkin_ws/src/path.csv");
 std::vector<std::string> classData;
 std::string line;

 while (std::getline(classFile, line,' ')) // there is input overload classfile
{
            classData.push_back(line);  
            std::cout<<line<<std::endl;
}


// hssh::LocalTopoGraph graph;

  // std::cout<<classData[0]<<std::endl;
  
  ros::spin();
  
  return 0;
}