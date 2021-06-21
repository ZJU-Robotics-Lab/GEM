/*
 * elevation_mapping_node.cpp
 *
 *  Created on: Octo 3, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */

#include <ros/ros.h>
#include <signal.h>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_mapping", ros::init_options::AnonymousName);

  ros::NodeHandle nodeHandle("~");
  elevation_mapping::ElevationMapping elevationMap(nodeHandle);

  boost::thread VisualThread(&elevation_mapping::ElevationMapping::visualPointMap, &elevationMap);
  boost::thread MapPublisherThread(&elevation_mapping::ElevationMapping::Run, &elevationMap);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
    
  return 0;
}
