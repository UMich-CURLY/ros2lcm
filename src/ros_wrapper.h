#ifndef ros_wrapper
#define ros_wrapper
// #include <hssh/metrical/data_queue.h>
// #include <hssh/metrical/data.h>
// #include <laser/line_extraction.h>
// #include <laser/line_extractor_params.h>
#include <core/imu_data.h>
#include <core/odometry.h>
// #include <core/laser_scan.h>
#include <core/motion_state.h>
#include <system/module_communicator.h>
// #include <utils/algorithm_ext.h>
// #include <utils/auto_mutex.h>
#include <utils/timestamp.h>
#include <cassert>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>

namespace vulcan
{
namespace system { class ModuleCommunicator; }
namespace hssh
{

class ros_wrapping
{
public:
ros::NodeHandle nh_;
ros::Subscriber sub_;

ros_wrapping();

}
}
}               

#endif // HSSH_UTILS_METRICAL_DATA_QUEUE_H
