/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
// Grid Map
#include <grid_map_msgs/GridMap.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

using namespace std;
using namespace grid_map;


namespace elevation_mapping {

ElevationMap::ElevationMap(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "min_height", "height", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "timestamp", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      visualMap_({"elevation", "variance", "intensity", "rough", "slope", "traver", "color_r", "color_g", "color_b"}),
      hasUnderlyingMap_(false),
      visibilityCleanupDuration_(0.0)
{
  rawMap_.setBasicLayers({"elevation", "variance"});
  visualMap_.setBasicLayers({"elevation"});

  clear();

  visualMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visual_map", 1);
  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  VpointsPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("visualpoints",1);
  orthomosaicPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("orthomosaic", 1);
  orthodepthPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("ortho_depth", 1);
  orthotraverPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("ortho_traver", 1);

  initialTime_ = ros::Time::now();
}


ElevationMap::~ElevationMap()
{
}


void ElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position)
{
  rawMap_.setGeometry(length, resolution, position);
  visualMap_.setGeometry(length, resolution, position);
}


void ElevationMap::show(int length, float *elevation, float *var, float *intensity, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, std_msgs::Header &header)
{
  cv::Mat image(length, length, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat depth_image(length, length, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat traver_image(length, length, CV_8UC3, cv::Scalar(0,0,0));

  visualMap_.clearAll();
  
  pointCloud show_pointCloud;
  anypoint show_point;
  int index, index_x, index_y;
  Index start_index = visualMap_.getStartIndex();

  for (GridMapIterator iterator(visualMap_); !iterator.isPastEnd(); ++iterator) {
    index_x = (*iterator).transpose().x();
    index_y = (*iterator).transpose().y();
    index = index_x * length + index_y;
    if(elevation[index] != -10)
    {
      visualMap_.at("elevation", *iterator) = elevation[index];
      visualMap_.at("variance", *iterator) = var[index];
      visualMap_.at("intensity", *iterator) = intensity[index];
      visualMap_.at("rough", *iterator) = rough[index];
      visualMap_.at("slope", *iterator) = slope[index];
      visualMap_.at("traver", *iterator) = traver[index];
      visualMap_.at("color_r", *iterator) = point_colorR[index];
      visualMap_.at("color_g", *iterator) = point_colorG[index];
      visualMap_.at("color_b", *iterator) = point_colorB[index];

      Position point;
      visualMap_.getPosition(*iterator, point);

      show_point.x = point.x();
      show_point.y = point.y();
      show_point.z = visualMap_.at("elevation", *iterator);
      show_point.r = visualMap_.at("color_r", *iterator);
      show_point.g = visualMap_.at("color_g", *iterator);
      show_point.b = visualMap_.at("color_b", *iterator);
      show_point.intensity = visualMap_.at("intensity", *iterator);
      image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[0] = visualMap_.at("intensity", *iterator) / 100 * 255;
      image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[1] = visualMap_.at("intensity", *iterator) / 100 * 255;
      image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[2] = visualMap_.at("intensity", *iterator) / 100 * 255;
      
      depth_image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[0] = (visualMap_.at("elevation", *iterator) + 7.0) / 10.0 * 255;
      depth_image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[1] = (visualMap_.at("elevation", *iterator) + 7.0) / 10.0 * 255;
      depth_image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[2] = (visualMap_.at("elevation", *iterator) + 7.0) / 10.0 * 255;
      
      traver_image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[0] = -1 * (visualMap_.at("traver", *iterator) - 1.0) * 255;
      traver_image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[1] = -1 * (visualMap_.at("traver", *iterator) - 1.0) * 255;
      traver_image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[2] = -1 * (visualMap_.at("traver", *iterator) - 1.0) * 255;
    
      show_pointCloud.push_back(show_point);
    }

  }
  pcl_conversions::toPCL(ros::Time::now(), show_pointCloud.header.stamp);
  show_pointCloud.header.frame_id = "robot0/map";
  
  if(show_pointCloud.size() > 0)
  {
    sensor_msgs::PointCloud2 pub_pointcloud;
    pcl::toROSMsg(show_pointCloud, pub_pointcloud);
    VpointsPublisher_.publish(pub_pointcloud); 
  }

  //cleanareaMap_.setTimestamp(timestamp.toNSec()); 
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(visualMap_, message);
  visualMapPublisher_.publish(message);
}

bool ElevationMap::clear()
{
  rawMap_.clearAll();
  rawMap_.resetTimestamp();
  visualMap_.clearAll();
  visualMap_.resetTimestamp();
  return true;
}


void ElevationMap::opt_move(Position M_position, float update_height)
{
  visualMap_.setPosition(M_position); 

  for (GridMapIterator iterator(visualMap_); !iterator.isPastEnd(); ++iterator) {
    if(visualMap_.at("elevation", *iterator) != -10)
    {
      visualMap_.at("elevation", *iterator) += update_height;
    }
  }
}


void ElevationMap::move(const Index M_startindex, Position M_position)
{
  visualMap_.setStartIndex(M_startindex);
  visualMap_.setPosition(M_position);
  
}


grid_map::GridMap& ElevationMap::getRawGridMap()
{
  return rawMap_;
}


ros::Time ElevationMap::getTimeOfLastUpdate()
{
  return ros::Time().fromNSec(rawMap_.getTimestamp());
}


const kindr::HomTransformQuatD& ElevationMap::getPose()
{
  return pose_;
}


void ElevationMap::setFrameId(const std::string& frameId)
{
  rawMap_.setFrameId(frameId);
  visualMap_.setFrameId(frameId);
}


const std::string& ElevationMap::getFrameId()
{
  return rawMap_.getFrameId();
}

} /* namespace */
