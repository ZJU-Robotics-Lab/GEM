/*
 * SensorProcessorBase.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/Core>

// STL
#include <unordered_map>
#include <string>
#include <memory>

#include "elevation_mapping/PointXYZRGBCE.hpp"
#include "elevation_mapping/PointXYZRGBC.hpp"
#include "elevation_mapping/PointXYZRGBI.hpp"

typedef PointXYZRGBI anypoint;
// typedef PointXYZRGBI anypoint;
typedef pcl::PointCloud<anypoint> pointCloud;
typedef pcl::PointCloud<anypoint>::Ptr pointCloudPtr;

namespace elevation_mapping {

/*!
* Generic Sensor processor base class. Provides functionalities
* common to all sensors and defines the interface for specialized
* sensor processor classes.
* Cleans the point cloud, transforms it to a desired frame, and
* computes the measurement variances based on a sensor model in
* the desired frame.
*/
class SensorProcessorBase
{
public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
	SensorProcessorBase(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

	/*!
	 * Destructor.
	 */
	virtual ~SensorProcessorBase();

	/*!
	 * Processes the point cloud.
	 * @param[in] pointCloudInput the input point cloud.
	 * @param[in] targetFrame the frame to which the point cloud should be transformed. // TODO Update.
	 * @param[out] pointCloudOutput the processed point cloud.
	 * @param[out] variances the measurement variances expressed in the target frame.
	 * @return true if successful.
	 */
  bool process(const pcl::PointCloud<anypoint>::ConstPtr pointCloudInput,
               const pcl::PointCloud<anypoint>::Ptr pointCloudOutput, 
               int *point_colorR,
               int *point_colorG,
               int *point_colorB,
               int *point_index,
               float *point_height,
               float *point_var,
               float *point_intensity);

  typedef std::unique_ptr<SensorProcessorBase> Ptr;

	friend class ElevationMapping;

  tf::StampedTransform M2StransformTf;
 protected:

  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
  virtual bool readParameters();

  /*!
   * Cleans the point cloud.
   * @param pointCloud the point cloud to clean.
   * @return true if successful.
   */
  virtual bool cleanPointCloud(const pcl::PointCloud<anypoint>::Ptr pointCloud) = 0;

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  virtual bool computeVariances(const pcl::PointCloud<anypoint>::ConstPtr pointCloud,
                                const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, Eigen::VectorXf& variances) = 0;

  
  
  /*!
   * Update the transformations for a given time stamp.
   * @param timeStamp the time stamp for the transformation.
   * @return true if successful.
   */
  bool updateTransformations(const ros::Time& timeStamp);


  bool GPUPointCloudprocess(
		pcl::PointCloud<anypoint>::Ptr pointCloudSensorframe,
		pcl::PointCloud<anypoint>::Ptr pointCloudTransformed,
		const std::string& targetFrame, 
    int *point_colorR,
    int *point_colorG,
    int *point_colorB,
    int *point_index,
    float *point_height,
    float *point_var,
    float *point_intensity);

     /*!
   * Transforms the point cloud the a target frame.
   * @param[in] pointCloud the point cloud to be transformed.
   * @param[out] pointCloudTransformed the resulting point cloud after transformation.
   * @param[in] targetFrame the desired target frame.
   * @return true if successful.
   */ 
  bool transformPointCloud(pcl::PointCloud<anypoint>::ConstPtr pointCloud,
                           pcl::PointCloud<anypoint>::Ptr pointCloudTransformed,
                           const std::string& targetFrame);

  void readcomputerparam(float *min_r, float *beam_a, float *beam_c, Eigen::RowVector3f *sensorJacobian, Eigen::Matrix3f *C_SB_transpose, Eigen::RowVector3f *P_mul_C_BM_transpose, Eigen::Matrix3f *B_r_BS_skew);
  
  /*!
   * Removes points with z-coordinate above a limit in map frame.
   * @param[in/out] pointCloud the point cloud to be cropped.
   */
  void removePointsOutsideLimits(pcl::PointCloud<anypoint>::ConstPtr reference,
                                 std::vector<pcl::PointCloud<anypoint>::Ptr>& pointClouds);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! TF transform listener.
  tf::TransformListener& transformListener_;

  //! The timeout duration for the lookup of the transformation between sensor frame and target frame.
  ros::Duration transformListenerTimeout_;

  //! Rotation from Base to Sensor frame (C_SB)
  kindr::RotationMatrixD rotationBaseToSensor_;

  //! Translation from Base to Sensor in Base frame (B_r_BS)
  kindr::Position3D translationBaseToSensorInBaseFrame_;

  //! Rotation from (elevation) Map to Base frame (C_BM)
  kindr::RotationMatrixD rotationMapToBase_;

  //! Translation from Map to Base in Map frame (M_r_MB)
  kindr::Position3D translationMapToBaseInMapFrame_;

  //! Transformation from Sensor to Map frame
  Eigen::Affine3d transformationSensorToMap_;

  

  //! TF frame id of the map.
  std::string mapFrameId_;

  //! TF frame id of the robot base.
  std::string robotBaseFrameId_;

  //! TF frame id of the range sensor for the point clouds.
  std::string sensorFrameId_;

  //! Ignore points above this height in map frame.
  double ignorePointsUpperThreshold_;

  //! Ignore points below this height in map frame.
  double ignorePointsLowerThreshold_;

  //! Sensor parameters.
  std::unordered_map<std::string, double> sensorParameters_;
};

} /* namespace elevation_mapping */
