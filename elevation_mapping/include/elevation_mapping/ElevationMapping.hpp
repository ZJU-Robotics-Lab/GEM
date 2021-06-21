/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"


// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h> 

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Empty.h>
#include <message_filters/sync_policies/approximate_time.h>

// Boost
#include <boost/thread.hpp>

// HASH GRID MAP
#include <math.h>
#include <assert.h>
#include <nav_msgs/Odometry.h>
#include <slam_msg/Keyframe.h>
#include <slam_msg/Keyframes.h>
#include <std_msgs/Bool.h>
#include <elevation_mapping/GridUtilHash.hpp>
#include <mutex>

using namespace pcl;
typedef unordered_map<const GridPoint, GridPointData, GridPointHashFunc, GridPointEqual> umap;
// typedef PointXYZRGBCE anypoint;
// typedef pcl::PointCloud<anypoint> pointCloud;
// typedef pcl::PointCloud<anypoint>::Ptr pointCloudPtr;
  
namespace elevation_mapping {
/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */


class ElevationMapping
{
//  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ElevationMapping(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  // HASH GRID MAP
  void updateLocalMap();
  void visualPointMap();
  void newKeyframeSignal(const nav_msgs::Odometry::ConstPtr& newKeyframeSignal);
  void mapSavingSignal(const std_msgs::Bool::ConstPtr& saveSignal);
  void fuseLocalMap(const slam_msg::Keyframes::ConstPtr& optKeyFrame);
  void Run();
  void calculate_memory_usage();

  // Util functions
  void localHashtoPointCloud(umap local_map, pointCloud::Ptr& outCloud);
  void pointCloudtoHash(pointCloud localPointCloud, umap& out);
  void gridMaptoPointCloud(grid_map::GridMap gridmap, pointCloud::Ptr& pc);
  void savingMap();

  /*!
   * Callback function for new data to be added to the elevation map.
   * @param pointCloud the point cloud to be fused with the existing data.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud);
  void Callback(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud);
  void processmapcells();
  void processpoints(pcl::PointCloud<anypoint>::ConstPtr pointCloud);
  

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */

  bool readParameters();

  /*!
   * Performs the initialization procedure.
   * @return true if successful.
   */
  bool initialize();

  /*!
   * Update the elevation map from the robot motion up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const ros::Time& time);

  /*!
   * Updates the location of the map to follow the tracking point. Takes care
   * of the data handling the goes along with the relocalization.
   * @return true if successful.
   */
  bool updateMapLocation();

  bool updatepointsMapLocation();
  /*!
   * Reset and start the map update timer.
   */
  void resetMapUpdateTimer();

  /*!
   * Stop the map update timer.
   */
  void stopMapUpdateTimer();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudProcessed;

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS subscribers.
  ros::Subscriber pointCloudSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! Callback thread for the fusion services.
  boost::thread fusionServiceThread_;

  //! Callback queue for fusion service thread.
  ros::CallbackQueue fusionServiceQueue_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;

  //! Gridmap scale.
  int length_;

  //! Gridmap resolution.
  double resolution_;

  //! Current location.
  float trackPointTransformed_x;
  float trackPointTransformed_y;
  float trackPointTransformed_z;

  ofstream Record_txt;
  double usage;
  double globalpoint_num;
  
  //! Some flags for trigger
  bool newLocalMapFlag;
  bool initFlag;
  bool optFlag;
  bool JumpFlag;
  bool insertionFlag;
  bool JumpOdomFlag;
  float* preopt_position;
  float prev_center_z;
  float position_shift[2];
  float current_position[2];
  float later_trackPointTransformed_z;

  string saving_file_;
  umap localMap_;
  pointCloud localPointMap_;
  pointCloud visualCloud_;
  pcl::PointCloud<PointXYZRGB> colorCloud_;
  vector<Eigen::Isometry3f> trajectory_;
  ros::Publisher pointMapPublisher_;
  ros::Publisher colorpointMapPublisher_;
  ros::Publisher globalMapPublisher_;
  ros::Publisher lastmapPublisher_;
  ros::Publisher savingSignalPublisher_;
  vector<pointCloud> globalMap_;
  // vector<pointCloud> visualGlobalMap_;
  vector<PointXY> localMapLoc_;
  vector<Eigen::Isometry3f> optGlobalMapLoc_;
  grid_map::Position prev_position;
  grid_map::GridMap prevMap_;
  ros::Subscriber keyFrameSignalSub_;
  ros::Subscriber optKeyframeSub_;
  ros::Subscriber fuseSignalSub_;
  ros::Subscriber savingSignalSub_;
  // ros::Subscriber pointCloudSubscriber_;
  pointCloud global_map;
  
  message_filters::Connection connection;
  message_filters::Subscriber<sensor_msgs::PointCloud2> vel_sub;
  message_filters::Subscriber<sensor_msgs::Image> cameraR_sub;
  // message_filters::Synchronizer<MySyncPolicy> sync;
  boost::recursive_mutex MapMutex_;
  std::mutex GlobalMapMutex_;
  std::mutex LocalMapMutex_;

  int robotPoseCacheSize_;

  //! TF listener and broadcaster.
  tf::TransformListener transformListener_;

  //! Point which the elevation map follows.
  kindr::Position3D trackPoint_;
  std::string trackPointFrameId_;

  //! ROS topics for subscriptions.
  std::string pointCloudTopic_;
  std::string robotPoseTopic_;

  //! Elevation map.
  ElevationMap map_;
  
  //! Sensor processors.
  SensorProcessorBase::Ptr sensorProcessor_;

  //! Robot motion elevation map updater.
  RobotMotionMapUpdater robotMotionMapUpdater_;

  //! If true, robot motion updates are ignored.
  bool ignoreRobotMotionUpdates_;

  //! Time of the last point cloud update.
  ros::Time lastPointCloudUpdateTime_;

  //! Timer for the robot motion update.
  ros::Timer mapUpdateTimer_;

  //! Maximum time that the map will not be updated.
  ros::Duration maxNoUpdateDuration_;

  //! Time tolerance for updating the map with data before the last update.
  //! This is useful when having multiple sensors adding data to the map.
  ros::Duration timeTolerance_;

  //! If map is fused after every change for debugging/analysis purposes.
  bool isContinouslyFusing_;

  float closeLoopZ;

};

} /* namespace */
