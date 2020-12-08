/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */
#include "elevation_mapping/ElevationMapping.hpp"

#include <thread>

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
// #include "elevation_mapping/Pointcloudtogrid.hpp"

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

//multi threads
#include <cstdlib>
#include <pthread.h>
#include <unistd.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/filters/passthrough.h>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Kindr
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>

using namespace std;
using namespace grid_map;
using namespace ros;
using namespace tf;
using namespace pcl;
using namespace kindr;
using namespace kindr_ros;

void Move(float *current_Position, float resolution, int length, float *h_central_coordinate, int *h_start_indice, float *position_shift);
void Init_GPU_elevationmap(int length, float resolution, float h_mahalanobisDistanceThreshold_, float h_obstacle_threshold);
void Map_closeloop(float *update_position, float height_update, int length, float resolution);
void Raytracing(int length_);
void Fuse(int length, int point_num, int *point_index, int *point_colorR, int *point_colorG, int *point_colorB, float *point_intensity, float *point_height, float *point_var);
void Map_feature(int length, float *elevation, float *var, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, float *intensity);
void Map_optmove(float *opt_p, float height_update, float resolution,  int length, float *opt_alignedPosition);
namespace elevation_mapping {

class ElevationMapping;

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle),
      gmap_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),
      isContinouslyFusing_(false),
      ignoreRobotMotionUpdates_(false),
      vel_sub(nodeHandle, "/voxel_grid/output", 1),
      cameraR_sub(nodeHandle, "/stereo_grey/left/image_raw", 1),
      sync(MySyncPolicy(10), vel_sub, cameraR_sub)
{
  ROS_INFO("Elevation mapping node started.");

  nodeHandle_.param("robot_id", robot_id, string("1"));
  nodeHandle_.param("robot_name", robot_name, string("robot1"));
  ROS_INFO("get robot_id: %s", robot_id.c_str());
  ROS_INFO("get robot_name: %s", robot_name.c_str());

  localMap_.rehash(10000);

  pointMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/history_point", 1);
  subMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/submap", 1);
  globalMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/global_point", 1);
  lastmapPublisher_ =  nodeHandle_.advertise<grid_map_msgs::GridMap>("opt_map", 1);
 
  readParameters();
  initialize();
}

void ElevationMapping::Run()
{
  savingSignalSub_ = nodeHandle_.subscribe("/" + robot_name + "/map_saving", 1, &ElevationMapping::mapSavingSignal, this);
}

void ElevationMapping::startsync()
{
  connection = sync.registerCallback(boost::bind(&ElevationMapping::Callback, this, _1, _2));
}

ElevationMapping::~ElevationMapping()
{
  nodeHandle_.shutdown();
}

bool ElevationMapping::readParameters()
{
  // ElevationMapping parameters.
  nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/robot"));
  nodeHandle_.param("camera_params_yaml", cameraParamsFile, string("/home/mav-lab/intrinsic.yaml"));
  nodeHandle_.param("robot_local_map_size", localMapSize_, 20.0);
  nodeHandle_.param("travers_threshold", traversThre, 0.0);
  nodeHandle_.param("track_point_x", trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", trackPoint_.z(), 0.0);

  nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  ROS_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  nodeHandle_.param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);
  
  // ElevationMap parameters. TODO Move this to the elevation map class.
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  map_.setFrameId(frameId);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  map_.setGeometry(length, resolution, position);
  
  nodeHandle_.param("map_saving_file", map_saving_file_, string("/home/mav-lab/slam_ws/test.pcd"));
  nodeHandle_.param("submap_saving_dir", submapDir, string("/home/mav-lab/slam_ws/"));

  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);
  nodeHandle_.param("underlying_map_topic", map_.underlyingMapTopic_, string());
  nodeHandle_.param("enable_visibility_cleanup", map_.enableVisibilityCleanup_, true);
  nodeHandle_.param("scanning_duration", map_.scanningDuration_, 1.0);

  float obstacle_threshold = 0.7;
  length_ = length(0) / resolution;
  resolution_ = resolution;

  Init_GPU_elevationmap(length_, resolution_, map_.mahalanobisDistanceThreshold_, obstacle_threshold);
 
  // SensorProcessor parameters.
  string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, string("structured_light"));
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, transformListener_));
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }

  if (!sensorProcessor_->readParameters()) return false;
  if (!robotMotionMapUpdater_.readParameters()) return false;

  return true;
}

bool ElevationMapping::initialize()
{
  ROS_INFO("Elevation mapping node initializing ... ");

  Duration(1.0).sleep(); // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  
  newLocalMapFlag = 1;
  initFlag = 1;
  JumpOdomFlag =0;
  ROS_INFO("Done.");
  return true;
}


void ElevationMapping::processpoints(pcl::PointCloud<Anypoint>::ConstPtr pointCloud)
{ 
  ros::Time begin_time = ros::Time::now ();

  int point_num = pointCloud->size();
  int point_index[point_num];
  float point_height[point_num];
  float point_var[point_num];
  
  int point_colorR[point_num];
  int point_colorG[point_num];
  int point_colorB[point_num];
  float point_intensity[point_num];

  PointCloud<Anypoint>::Ptr pointProcessed(new PointCloud<Anypoint>);

  if (!this->sensorProcessor_->process(pointCloud, pointProcessed, point_colorR, point_colorG, point_colorB, point_index, point_intensity, point_height, point_var)) {
    ROS_ERROR("Point cloud could not be processed.");
    this->resetMapUpdateTimer();
  }

  boost::recursive_mutex::scoped_lock lock(MapMutex_);
  Fuse(length_, point_num, point_index, point_colorR, point_colorG, point_colorB, point_intensity, point_height, point_var);
  lock.unlock();
}

void ElevationMapping::processmapcells()
{
  ros::Time begin_time = ros::Time::now ();
  boost::recursive_mutex::scoped_lock lock(MapMutex_);

  if (!this->updatePrediction(this->lastPointCloudUpdateTime_)) {
    ROS_ERROR("Updating process noise failed.");
    this->resetMapUpdateTimer();
    return;
  }
  lock.unlock();   
}

Eigen::Matrix<double,4,4> toMatrix44(const cv::Mat &cvMat)
{
    Eigen::Matrix<double,4,4> M;

    M << cvMat.at<double>(0,0), cvMat.at<double>(0,1), cvMat.at<double>(0,2), cvMat.at<double>(0,3),
    cvMat.at<double>(1,0), cvMat.at<double>(1,1), cvMat.at<double>(1,2), cvMat.at<double>(1,3),
    cvMat.at<double>(2,0), cvMat.at<double>(2,1), cvMat.at<double>(2,2), cvMat.at<double>(2,3),
    cvMat.at<double>(3,0), cvMat.at<double>(3,1), cvMat.at<double>(3,2), cvMat.at<double>(3,3);

    return M;
}

Eigen::Matrix<double,3,4> toMatrix34(const cv::Mat &cvMat)
{
    Eigen::Matrix<double,3,4> M;

    M << cvMat.at<double>(0,0), cvMat.at<double>(0,1), cvMat.at<double>(0,2), cvMat.at<double>(0,3),
    cvMat.at<double>(1,0), cvMat.at<double>(1,1), cvMat.at<double>(1,2), cvMat.at<double>(1,3),
    cvMat.at<double>(2,0), cvMat.at<double>(2,1), cvMat.at<double>(2,2), cvMat.at<double>(2,3);

    return M;
}

void ElevationMapping::Callback(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud, const sensor_msgs::Image::ConstPtr& image)
{
  ros::Time begin_time = ros::Time::now ();
  sensor_msgs::PointCloud2 output = *rawPointCloud;
  pcl::PCLPointCloud2 Point_cloud;
  pcl_conversions::toPCL(output, Point_cloud);
  pcl::PointCloud<Anypoint>::Ptr pointCloud(new pcl::PointCloud<Anypoint>);
  pcl::fromPCLPointCloud2(Point_cloud, *pointCloud);
  // pcl::PointCloud<Anypoint> Color_pointCloud;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr -> image;
  
  int dev = 64;
  Eigen::Vector4d P_lidar;
  Eigen::Vector3d P_img;
  Eigen::Vector3d P_XY;
  float P_x, P_y;
  Eigen::MatrixXd P_lidar2img(3, 4);
  Eigen::MatrixXd Tcamera(3, 4);
  Eigen::MatrixXd TLidar(4, 4);
  cv::Mat TCAM, TV;

  // Read camera instrinsic and camera-lidar extrinsics parameters
  cv::FileStorage fsSettings(cameraParamsFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
  }
  fsSettings["T.camera"] >> TCAM;
  fsSettings["T.lidar"] >> TV;
  Tcamera = toMatrix34(TCAM);
  TLidar = toMatrix44(TV);

  P_lidar2img = Tcamera * TLidar;

  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);

  // Project image to point cloud
  for(int i = 0; i < pointCloud->points.size(); i++)
  {
      P_lidar << pointCloud->points[i].x, 
                pointCloud->points[i].y,
                pointCloud->points[i].z,
                1;
      P_img = P_lidar2img * P_lidar;
      
      P_x = P_img.x() / P_img.z();
      P_y = P_img.y() / P_img.z();

      cv::Point midPoint;

      midPoint.x = P_x;
      midPoint.y = P_y;
      if(midPoint.x > 0 && midPoint.x < 1241 && midPoint.y > 0 && midPoint.y < 376)
      {   
          int b = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[0];
          int g = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[1];
          int r = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[2];
          cv::circle(img, midPoint, 1, cv::Scalar(b, g, r));
          pointCloud->points[i].b = b;
          pointCloud->points[i].g = g;
          pointCloud->points[i].r = r;
          // Color_pointCloud.push_back(pointCloud->points[i]);
      }
      else
      {
          pointCloud->points[i].b = 0;
          pointCloud->points[i].g = 0;
          pointCloud->points[i].r = 0;
          pointCloud->points[i].i = 0;
      }
  }

  // Color_pointCloud.header = pointCloud->header;
  
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  ROS_INFO("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));
  sensorProcessor_->updateTransformations(timeStamp);
  updatepointsMapLocation();
  updateMapLocation();

  std::thread t1(&ElevationMapping::processpoints, this, pointCloud);
  std::thread t2(&ElevationMapping::processmapcells, this);
  t1.join();
  t2.join();  

  float elevation[length_ * length_];
  int point_colorR[length_ * length_];
  int point_colorG[length_ * length_];
  int point_colorB[length_ * length_];
  float rough[length_ * length_];
  float slope[length_ * length_];
  float traver[length_ * length_];
  float var[length_ * length_];
  float intensity[length_ * length_];

  Map_feature(length_, elevation, var, point_colorR, point_colorG, point_colorB, rough, slope, traver, intensity);
  // for(auto i = 0; i < length_ * length_; i++){
  //   cout << "point intensity " << intensity[i] << endl;
  // }
  map_.show(timeStamp, robot_name, trackPointTransformed_x, trackPointTransformed_y, length_, elevation, var, point_colorR, point_colorG, point_colorB, rough, slope, traver, intensity);

  updateLocalMap();
  visualPointMap();

  Raytracing(length_);
  prevMap_ = map_.visualMap_;
  // calculate_memory_usage();
}

void ElevationMapping::calculate_memory_usage()
{
    double usgae_KB = 0;
    usgae_KB += sizeof(decltype(globalMap_.back())) * globalMap_.capacity(); 
    usgae_KB += sizeof(decltype(optGlobalMapLoc_.back())) * optGlobalMapLoc_.capacity(); 
    // usgae_KB += inactive_pointcloud->size() * sizeof(PointType) / 1024.0;
    // printf("the process comsumes %f KB\n", usgae_KB);
}

void ElevationMapping::savingMap()
{
  gmap_.setFrameId("/" + robot_name + "/map");

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  length(0) = 200;
  length(1) = 200;
  position.x() = 0.0;
  position.y() = 0.0;
  resolution = 0.1;
  gmap_.setGeometry(length, resolution, position);

  gmap_.minVariance_= pow(0.003, 2);
  gmap_.maxVariance_, pow(0.03, 2);

  pcl::PointCloud<Anypoint> out_color;
  ROS_INFO("NUM: %d", visualCloud_.size());

  pcl::PointCloud<Anypoint>::Ptr cloud_in(new pcl::PointCloud<Anypoint>);
  pcl::PointCloud<Anypoint>::Ptr cloud_filtered(new pcl::PointCloud<Anypoint>);

  for(int i = 0; i < visualCloud_.size(); i++){
    Anypoint pt;
    pt.x = visualCloud_.points[i].x;
    pt.y = visualCloud_.points[i].y;
    pt.z = visualCloud_.points[i].z;
    pt.r = visualCloud_.points[i].r;
    pt.g = visualCloud_.points[i].g;
    pt.b = visualCloud_.points[i].b;
    pt.i = visualCloud_.points[i].i;
    pt.covariance = visualCloud_.points[i].covariance;
    out_color.push_back(pt);
  }

  ROS_INFO("Saving Map to %s", map_saving_file_);
  pcl::io::savePCDFile(map_saving_file_, out_color);
}

void ElevationMapping::savingSubMap()
{
  ROS_WARN("SUB MAP NUM: %d", globalMap_.size());

  std::string currentName_;
  pointCloud cloudpt;

  for(int i = 0; i < globalMap_.size(); i++){
    cloudpt = globalMap_[i];
    std::ostringstream cc;
    cc << i;
    currentName_ = (submapDir + cc.str() + ".pcd").c_str();
    ROS_INFO("Saving Map to %s", currentName_);
    pcl::io::savePCDFile(currentName_, cloudpt);
  }

}

void ElevationMapping::visualPointMap()
{
  pcl::PointCloud<Anypoint>::Ptr cloud_in(new pcl::PointCloud<Anypoint>);
  cloud_in = visualCloud_.makeShared();

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(visualCloud_, output);
  output.header.frame_id = "/" + robot_name + "/map";
  pointMapPublisher_.publish(output);

}

void ElevationMapping::mapSavingSignal(const std_msgs::Bool::ConstPtr& savingSignal)
{
  if(savingSignal->data == 1)
    savingMap();
    savingSubMap();
  ROS_WARN("Saving Global Map at: %s.", map_saving_file_.c_str());
}

void ElevationMapping::localHashtoPointCloud(umap localMap, pointCloud::Ptr& outCloud)
{
  pointCloud hashPointCloud;
  for(auto it = localMap.begin(); it != localMap.end(); it++){
    Anypoint pt;
    pt.x = it->first.x;
    pt.y = it->first.y;
    pt.z = it->second.elevation;
    pt.r = it->second.r;
    pt.g = it->second.g;
    pt.b = it->second.b;
    pt.covariance = it->second.var;
    hashPointCloud.push_back(pt);
  }
  outCloud = hashPointCloud.makeShared();
}

void ElevationMapping::pointCloudtoHash(pointCloud localPointCloud, umap& out)
{
  for(size_t i = 0; i < localPointCloud.size (); ++i){
    float round_x, round_y, round_z;
    round_x = (ceil(localPointCloud.points[i].x / resolution_)) * resolution_ - resolution_ / 2.0;
    round_y = (ceil(localPointCloud.points[i].y / resolution_)) * resolution_ - resolution_ / 2.0;
    round_z = localPointCloud.points[i].z;

    GridPoint save_pos(round_x, round_y);
    GridPointData save_data(round_z, localPointCloud.points[i].covariance, localPointCloud.points[i].r, localPointCloud.points[i].g, localPointCloud.points[i].b, localPointCloud.points[i].i);
    out.insert(make_pair(save_pos, save_data));
  }
}

void ElevationMapping::gridMaptoPointCloud(grid_map::GridMap gridmap, pointCloud::Ptr& pc)
{
}

void ElevationMapping::updateLocalMap()
{
  int index, index_x, index_y;
  float delta_x, delta_y;
  double t2;

  float current_x = current_position[0];
  float current_y = current_position[1];

  delta_x = position_shift[0];
  delta_y = position_shift[1];

  tf::StampedTransform trackPoseTransformed_;
  try{
    transformListener_.lookupTransform("/" + robot_name + "/map", trackPointFrameId_, ros::Time(0), trackPoseTransformed_);
    // ROS_WARN("tf********* x: %lf, y: %lf, z: %lf",trackPoseTransformed_.getOrigin().x(), trackPoseTransformed_.getOrigin().y(), trackPoseTransformed_.getOrigin().z());
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  if(initFlag == 0 && sqrt(pow((trackPoseTransformed_.getOrigin().x() - trajectory_.back().translation().x()),2) + pow((trackPoseTransformed_.getOrigin().y() - trajectory_.back().translation().y()),2)) >= localMapSize_){
    newLocalMapFlag = 1;
    ROS_WARN("NEW KEYFRAME ****************");
  }
  
  if(JumpFlag == 1){
    pointCloud cloudUpdated;
    pointCloud::Ptr updatedLocalMap;
    localHashtoPointCloud(localMap_, updatedLocalMap);

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T = optGlobalMapLoc_[globalMap_.size()] * trajectory_[globalMap_.size()].inverse();
    Eigen::Matrix4f transformMatrix = T.matrix();
    pcl::transformPointCloud(*updatedLocalMap, cloudUpdated, transformMatrix);  

    visualCloud_ += cloudUpdated;
  }

  if(newLocalMapFlag == 1 && (JumpFlag == 1 || optFlag == 0)){// && JumpOdomFlag == 0){ // At the local_map's boundary
    
    if(!localMap_.empty() && initFlag == 0){ // Not the init state

      // Get keyframe pose
      Eigen::Quaternionf q(trackPoseTransformed_.getRotation().w(), trackPoseTransformed_.getRotation().x(), trackPoseTransformed_.getRotation().y(), trackPoseTransformed_.getRotation().z());
      Eigen::Isometry3f prev_center(q);
      prev_center.pretranslate(Eigen::Vector3f(trackPoseTransformed_.getOrigin().x(), trackPoseTransformed_.getOrigin().y(), trackPoseTransformed_.getOrigin().z()));
      trajectory_.push_back(prev_center);

      // Save the center of every keyframe
      PointXY localMapCenter_;
      localMapCenter_.x = trajectory_.back().translation().x();
      localMapCenter_.y = trajectory_.back().translation().y();
      localMapLoc_.push_back(localMapCenter_);
      pointCloud::Ptr out_pc;

      // Save local map into global map stack
      localHashtoPointCloud(localMap_, out_pc);
      unique_lock<mutex> lock(GlobalMapMutex_);
      globalMap_.push_back(*out_pc);
      lock.unlock();

      // Publish submap
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*out_pc, output);
      output.header.frame_id = "/" + robot_name + "/map";
      subMapPublisher_.publish(output);

      // Quick clear unordered_map
      umap tmp;
      localMap_.swap(tmp);
      newLocalMapFlag = 0;

    }else if(initFlag == 1){  // Init process
      Eigen::Isometry3f prev_center = Eigen::Isometry3f::Identity();  // Initial map center
      prev_center.pretranslate(Eigen::Vector3f(0.0, 0.0, 0.0));
      trajectory_.push_back(prev_center);

      PointXY localMapCenter_;
      localMapCenter_.x = trajectory_.back().translation().x();
      localMapCenter_.y = trajectory_.back().translation().y();
      localMapLoc_.push_back(localMapCenter_);
      umap tmp;
      localMap_.swap(tmp);
      
      optFlag = 0;
      initFlag = 0;
      JumpOdomFlag = 0;
      JumpFlag = 0;
      
      newLocalMapFlag = 0;
      ROS_ERROR("Init......");
      return;
    }
    
  }

  ROS_INFO("Move_x: %lf, Move_y: %lf ",delta_x, delta_y);
  int count = 0;

  // Local mapping
  if(abs(delta_x) >= resolution_ || abs(delta_y) >= resolution_ && initFlag == 0 && JumpFlag == 0){// && (JumpOdomFlag == 0 || newLocalMapFlag == 0)){
    for (GridMapIterator iterator(prevMap_); !iterator.isPastEnd(); ++iterator) {   // Add L shape infomation of the previous grid map into the local map

      grid_map::Position position;
      prevMap_.getPosition(*iterator, position);
      index_x = (*iterator).transpose().x();
      index_y = (*iterator).transpose().y();
      index = index_x * length_ + index_y;

      if(prevMap_.at("elevation", *iterator) != -10 && prevMap_.at("traver", *iterator) >= traversThre){  //TODO: Param, save the grid information with specific feature to the local map
       if(((position.x() < (current_x - length_ * resolution_ / 2) || position.y() < (current_y - length_ * resolution_ / 2 )) && (delta_x > 0  && delta_y > 0))
           || ((position.x() > (current_x + length_ * resolution_ / 2 ) || position.y() > (current_y + length_ * resolution_ / 2 )) && (delta_x < 0 && delta_y < 0))
           || ((position.x() < (current_x - length_ * resolution_ / 2 ) || position.y() > (current_y + length_ * resolution_ / 2 )) && (delta_x > 0 && delta_y < 0))
           || ((position.x() > (current_x + length_ * resolution_ / 2 ) || position.y() < (current_y - length_ * resolution_ / 2 )) && (delta_x < 0 && delta_y > 0))
           || ((position.x() < (current_x - length_ * resolution_ / 2 )) && (delta_x > 0 && delta_y == 0))
           || ((position.x() > (current_x + length_ * resolution_ / 2 )) && (delta_x < 0 && delta_y == 0))
           || ((position.y() < (current_y - length_ * resolution_ / 2 )) && (delta_y > 0 && delta_x == 0))
           || ((position.y() > (current_y + length_ * resolution_ / 2 )) && (delta_y < 0 && delta_x == 0))){ 
            
            GridPoint save_pos(position.x(), position.y());
            GridPointData save_data(prevMap_.at("elevation", *iterator), prevMap_.at("variance", *iterator), prevMap_.at("color_r", *iterator), 
                                    prevMap_.at("color_g", *iterator), prevMap_.at("color_b", *iterator), prevMap_.at("intensity", *iterator));
            
            // Update the grid information within local map
            auto got = localMap_.find(save_pos);
            if(got == localMap_.end()){
              localMap_.insert(make_pair(save_pos, save_data));
            }else{
              localMap_.erase(got);
              localMap_.insert(make_pair(save_pos, save_data));
              count++;
            }

            // Add information to real-time visualization
            Anypoint pt;
            pt.x = position.x();
            pt.y = position.y();
            pt.z = prevMap_.at("elevation", *iterator);
            pt.r = prevMap_.at("color_r", *iterator);
            pt.g = prevMap_.at("color_g", *iterator);
            pt.b = prevMap_.at("color_b", *iterator);
            pt.i = prevMap_.at("intensity", *iterator);
            pt.covariance = prevMap_.at("variance", *iterator);
            visualCloud_.push_back(pt);
        }
      }
    }
    // ROS_WARN("eliminating same position point %d", count);
  }
  JumpFlag = 0;
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  if (ignoreRobotMotionUpdates_) return true;

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());
 
  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
   // map_.ElevationMap::getRawGridMap().setTimestamp(0);
  }
  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  HomTransformQuatD robotPose;
  geometry_msgs::Pose Tf_robotpose;

  Tf_robotpose.position.x = sensorProcessor_->M2StransformTf.getOrigin().x();
	Tf_robotpose.position.y = sensorProcessor_->M2StransformTf.getOrigin().y();
	Tf_robotpose.position.z = sensorProcessor_->M2StransformTf.getOrigin().z();
	Tf_robotpose.orientation.x = sensorProcessor_->M2StransformTf.getRotation().getX();
  Tf_robotpose.orientation.y = sensorProcessor_->M2StransformTf.getRotation().getY();
  Tf_robotpose.orientation.z = sensorProcessor_->M2StransformTf.getRotation().getZ();
  Tf_robotpose.orientation.w = sensorProcessor_->M2StransformTf.getRotation().getW();
	// std::cout << "Tf_robotpose " << Tf_robotpose.position.x << "  "<< Tf_robotpose.position.y << std::endl; 
  convertFromRosGeometryMsg(Tf_robotpose, robotPose);

  if(Tf_robotpose.position.x == 0 && Tf_robotpose.position.y == 0)
    return true;
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
                        
  // Compute map variance update from motio n prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);
   
  return true;
}

bool ElevationMapping::updatepointsMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = ros::Time(0);
  convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  trackPointTransformed_x = trackPointTransformed.point.x;
  trackPointTransformed_y = trackPointTransformed.point.y;
  trackPointTransformed_z = trackPointTransformed.point.z;
}

bool ElevationMapping::updateMapLocation()
{
  float current_p[3];
  current_p[0] = trackPointTransformed_x;
  current_p[1] = trackPointTransformed_y;
  current_p[2] = trackPointTransformed_z;
  grid_map::Index M_startindex;
  grid_map::Position M_position;
  
  if(JumpOdomFlag == 1)
  {
    tf::StampedTransform trackPoseTransformed_;
    tf::StampedTransform map2vel;
    
    geometry_msgs::PointStamped trackPointTransformed;

    try{
      transformListener_.lookupTransform("/" + robot_name + "/map", "/" + robot_name + "/Local", ros::Time(0), trackPoseTransformed_);  // TODO: Param
      
    }catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::StampedTransform map2sensor;
    try{
      transformListener_.lookupTransform("/" + robot_name + "/map", trackPointFrameId_, ros::Time(0), map2sensor);  // TODO: Param
      ROS_WARN("map2sensor____________********* %lf", map2sensor.getOrigin().z());
    }catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    float sensor_Z = map2sensor.getOrigin().z();

    // closeLooptime
    // float detal_x = trackPoseTransformed_.getOrigin().x() - trackPointTransformed_x;
    // float detal_y = trackPoseTransformed_.getOrigin().y() - trackPointTransformed_y;
    float height_update = trackPointTransformed_z - later_trackPointTransformed_z;

    if(sensor_Z != closeLoopZ)
    {
      pointCloud cloudUpdated;
      pointCloud::Ptr updatedLocalMap;
      localHashtoPointCloud(localMap_, updatedLocalMap);

      Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
      T = optGlobalMapLoc_[globalMap_.size()] * trajectory_[globalMap_.size()].inverse();
      Eigen::Matrix4f transformMatrix = T.matrix();
      pcl::transformPointCloud(*updatedLocalMap, cloudUpdated, transformMatrix);  

      visualCloud_ += cloudUpdated;
      
      float opt_position[2];
      float opt_alignedPosition[2];
      
      geometry_msgs::PointStamped trackPoint;
      trackPoint.header.frame_id = "/" + robot_name + "/Local";
      trackPoint.header.stamp = ros::Time(0);
      convertToRosGeometryMsg(trackPoint_, trackPoint.point);
      geometry_msgs::PointStamped trackPointTransformed;
      trackPoint.point.x -= 2 * resolution_;
      try {
        transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
      } catch (TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return false;
      }

      opt_position[0] = trackPointTransformed.point.x;
      opt_position[1] = trackPointTransformed.point.y;
      height_update = trackPointTransformed_z - later_trackPointTransformed_z;

      Map_optmove(opt_position, height_update, resolution_,  length_, opt_alignedPosition);
      M_position.x() = opt_alignedPosition[0];
      M_position.y() = opt_alignedPosition[1];
      map_.opt_move(M_position, height_update);
      prevMap_ = map_.visualMap_;
      //Map_closeloop(update, height_update, length_, resolution_);
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(prevMap_, message);
      lastmapPublisher_.publish(message);
      JumpOdomFlag =0;
      JumpFlag = 1;
    }
  }

  int d_startindex[2];

  Move(current_p , resolution_,  length_, current_position, d_startindex, position_shift);
  
  M_startindex.x() = d_startindex[0];
  M_startindex.y() = d_startindex[1];

  M_position.x() = current_position[0];
  M_position.y() = current_position[1];
  
  map_.move(M_startindex, M_position);
  later_trackPointTransformed_z = trackPointTransformed_z;
  return true;
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace */
