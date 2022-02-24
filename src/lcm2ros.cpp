/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     data_queue.cpp
* \author   Collin Johnson
*
* Definition of MetricSLAMDataQueue.
*/
#include <hssh/local_metric/lpm_io.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/pose.h>

#include <hssh/local_topological/voronoi_skeleton_grid.h>

#include <vision/image_utils.h>
#include <visualization_msgs/Marker.h>
// #include <hssh/metrical/data_queue.h>
// #include <hssh/metrical/data.h>
// #include <laser/line_extraction.h>
// #include <laser/line_extractor_params.h>
#include <core/imu_data.h>
#include <core/odometry.h>
#include <core/landmark.h>
// #include <core/point.h>
// #include <core/laser_scan.h>
#include <core/motion_state.h>
#include <system/module_communicator.h>
// #include <utils/algorithm_ext.h>
// #include <utils/auto_mutex.h>
#include <hssh/local_topological/local_topo_map.h>
#include <nav_msgs/GetMap.h>
#include <utils/timestamp.h>
#include <cassert>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

namespace vulcan
{
namespace system { class ModuleCommunicator; }
namespace hssh
{

class lcm2ros
{
public:
ros::NodeHandle VoroNode;

ros::Publisher pub_map;

system::ModuleCommunicator* communicator;

lcm2ros()
{
    ros::NodeHandle private_nh("~");
    // if(!lcm.good())
    //     ros::shutdown();

    std::cout<<"Init rosnode"<<std::endl;
    pub_map = VoroNode.advertise<nav_msgs::OccupancyGrid>("/voronoi_map",1000);
    std::cout<<"Init publisher"<<std::endl;
    // subscribe(*communicator);
}

~lcm2ros()
{

}

// void handleData(const hssh::VoronoiSkeletonGrid& grid, const std::string& channel);
void handleData(const hssh::LocalTopoMap& topoMap, const std::string& channel);
void subscribe(system::ModuleCommunicator& producer);
};

void lcm2ros::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalTopoMap>(this);
    std::cout<<"Init subscriber"<<std::endl;
}


void lcm2ros::handleData(const hssh::LocalTopoMap& topoMap, const std::string& channel)
{
    std::cout<<"\n\nStart VORONOI Map!!!\n\n"<<std::endl;
    VoronoiSkeletonGrid grid = topoMap.voronoiSkeleton();

    nav_msgs::OccupancyGrid ros_grid;

    ros_grid.header.frame_id = grid.getReferenceIndex();
    ros_grid.header.stamp = ros::Time::now();
    ros_grid.info.resolution = grid.cellsPerMeter();

    ros_grid.info.width = grid.getWidthInCells();
    ros_grid.info.height = grid.getHeightInCells();


    ros_grid.info.origin.position.x = grid.getGlobalCenter().x;
    ros_grid.info.origin.position.y = grid.getGlobalCenter().y;
    // grid.info.origin.position.z = 0.0;
    // grid.info.origin.orientation.w = 1.0;

    ros_grid.data.resize(ros_grid.info.width * ros_grid.info.height);

    ros_grid.data = {0};

    for(auto& cell : grid.filtered_cells(grid.getJunctionsPoints(), 0x40, grid.getClassificationGrid())){
        ros_grid.data[cell.x + cell.y*ros_grid.info.width ] = 128;
    }

    std::cout<<"\n\nGet VORONOI Map!!!\n\n"<<std::endl;
    pub_map.publish(ros_grid);

}


}
}

int main(int argc, char** argv)
{
  using namespace vulcan;
  using namespace vulcan::system;
  using namespace vulcan::hssh;
  ros::init(argc, argv, "lcm2ros_node");
  
  system::ModuleCommunicator* communicator = new system::ModuleCommunicator();
  lcm2ros roswrapper;
  roswrapper.subscribe(*communicator);
  // roswrapper.subscribe(communicator);

  // visualization_msgs::Marker marker_test;
  // marker_test.pose.position.x = 700;
  // marker_test.pose.position.y = 600;
  // ros::Rate loop_rate(10);
  // while(ros::ok()){
  //     roswrapper.pub_pose.publish(marker_test);
  //     loop_rate.sleep();
  // }
  ros::spin();
  
  return 0;
}
