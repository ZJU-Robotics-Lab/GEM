/*
 * elevation_mapping_node.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */

#include <ros/ros.h>
#include <signal.h>
#include "elevation_mapping/ElevationMapping.hpp"


/*
* Check some parameters' format
*/
void checkFormat(std::string& robotName)
{
  ROS_INFO("Check Format");
  // check format
  std::string slash = "/";
  if((robotName.find(slash)) == string::npos && !robotName.empty()){
    robotName = "/" + robotName;
  }

  ROS_INFO("Check Format Done");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_mapping", ros::init_options::AnonymousName);

  ros::NodeHandle nodeHandle("~");

  string robot_id;
  string robot_name;

  nodeHandle.param("robot_id", robot_id, string("1"));
  nodeHandle.param("robot_name", robot_name, string("robot1"));
  ROS_INFO("get robot_id: %s", robot_id.c_str());
  ROS_INFO("get robot_name: %s", robot_name.c_str());

  checkFormat(robot_name);

  elevation_mapping::ElevationMapping elevationMap(nodeHandle, robot_name);

  boost::thread MapPublisherThread(&elevation_mapping::ElevationMapping::Run, &elevationMap);
  boost::thread MapComposingThread(&elevation_mapping::ElevationMapping::composingGlobalMapThread, &elevationMap);
  boost::thread LoopCloseThread(&elevation_mapping::ElevationMapping::updateGlobalMap, &elevationMap);

  ros::MultiThreadedSpinner spinner(4);
  elevationMap.startsync();
  
  spinner.spin();
  
  return 0;
}
