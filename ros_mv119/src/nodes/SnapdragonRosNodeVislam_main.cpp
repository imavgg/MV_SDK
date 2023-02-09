
#include "SnapdragonRosNodeVislam.hpp"
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) {
//Initializes ROS, and sets up a node
  ros::init(argc,argv,"SnapdragonVislam");
  ros::NodeHandle nh;
  Snapdragon::RosNode::Vislam vislam(nh);

  ros::spin();
  vislam.Stop();
  return 0;
}