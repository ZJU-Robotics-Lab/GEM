/*
 * StructuredLightSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <limits>
#include <string>

//int structlight_process(float *h_result, float *point_x, float *point_y, float *point_z, int point_num, float factor_a, float factor_b, float factor_c, float factor_d, float factor_e, float lateral_factor, Eigen::RowVector3f sensorJacobian, Eigen::Matrix3f rotationVariance, Eigen::Matrix3f C_SB_transpose, Eigen::RowVector3f P_mul_C_BM_transpose, Eigen::Matrix3f B_r_BS_skew);
namespace elevation_mapping {

/*! StructuredLight-type (structured light) sensor model:
 * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance - sensorModelNormalFactorC_)^2;
 * standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance
 * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
 */

StructuredLightSensorProcessor::StructuredLightSensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{

}

StructuredLightSensorProcessor::~StructuredLightSensorProcessor()
{

}

bool StructuredLightSensorProcessor::readParameters()
{
  SensorProcessorBase::readParameters();
  nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], std::numeric_limits<double>::min());
  nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], std::numeric_limits<double>::max());
  nodeHandle_.param("sensor_processor/normal_factor_a", sensorParameters_["normal_factor_a"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_b", sensorParameters_["normal_factor_b"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_c", sensorParameters_["normal_factor_c"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_d", sensorParameters_["normal_factor_d"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_e", sensorParameters_["normal_factor_e"], 0.0);
  nodeHandle_.param("sensor_processor/lateral_factor", sensorParameters_["lateral_factor"], 0.0);
  return true;
}

bool StructuredLightSensorProcessor::cleanPointCloud(const pcl::PointCloud<anypoint>::Ptr pointCloud)
{
	pcl::PassThrough<anypoint> passThroughFilter;
	pcl::PointCloud<anypoint> tempPointCloud;

	passThroughFilter.setInputCloud(pointCloud);
	passThroughFilter.setFilterFieldName("z");
	passThroughFilter.setFilterLimits(sensorParameters_.at("cutoff_min_depth"), sensorParameters_.at("cutoff_max_depth"));
	// This makes the point cloud also dense (no NaN points).
	passThroughFilter.filter(tempPointCloud);
	tempPointCloud.is_dense = true;
	pointCloud->swap(tempPointCloud);

	ROS_DEBUG("cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
	return true;
}

bool StructuredLightSensorProcessor::computeVariances(
		const pcl::PointCloud<anypoint>::ConstPtr pointCloud,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		Eigen::VectorXf& variances)
{
	variances.resize(pointCloud->size());

	// Projection vector (P).
	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

	// Sensor Jacobian (J_s).
	const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

	// Robot rotation covariance matrix (Sigma_q).
	Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

	float factor_a = sensorParameters_.at("normal_factor_a");
	float factor_b = sensorParameters_.at("normal_factor_b");
	float factor_c = sensorParameters_.at("normal_factor_c");
	float factor_d = sensorParameters_.at("normal_factor_d");
	float factor_e = sensorParameters_.at("normal_factor_e");
    float lateral_factor = sensorParameters_.at("lateral_factor");              

	int point_num = pointCloud->size();
	float point_x[point_num];
	float point_y[point_num];
	float point_z[point_num];
    float cuda_var[point_num];
	for (size_t i = 0; i < point_num; ++i) {
		auto& point = pointCloud->points[i];
        point_x[i] = point.x;
		point_y[i] = point.y;
		point_z[i] = point.z;
	} 
	
	//Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
	//cloud_device.upload(Cloud.points);
	//
	//func();
	
    ros::Time begin_time = ros::Time::now ();
	//structlight_process(cuda_var, point_x, point_y, point_z, point_num, factor_a, factor_b, factor_c, factor_d, factor_e, lateral_factor, sensorJacobian, rotationVariance, C_SB_transpose, P_mul_C_BM_transpose, B_r_BS_skew);
  	double t2 = (ros::Time::now () - begin_time).toSec ();
	for(size_t i = 0; i < point_num; ++i){
		variances(i) = cuda_var[i];
	}

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
		// For every point in point cloud.

		// Preparation.
		auto& point = pointCloud->points[i];
		Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
		float heightVariance = 0.0; // sigma_p

		// Measurement distance.
		float measurementDistance = pointVector.z();

		// Compute sensor covariance matrix (Sigma_S) with sensor model.
                float deviationNormal = sensorParameters_.at("normal_factor_a")
                    + sensorParameters_.at("normal_factor_b")
                        * (measurementDistance - sensorParameters_.at("normal_factor_c")) * (measurementDistance - sensorParameters_.at("normal_factor_c"))
                    + sensorParameters_.at("normal_factor_d") * pow(measurementDistance, sensorParameters_.at("normal_factor_e"));
		float varianceNormal = deviationNormal * deviationNormal;
		float deviationLateral = sensorParameters_.at("lateral_factor") * measurementDistance;
		float varianceLateral = deviationLateral * deviationLateral;
		Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
		sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

		// Robot rotation Jacobian (J_q).
		const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
		Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

		// Measurement variance for map (error propagation law).
		heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
		heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

		// Copy to list.
		variances(i) = heightVariance;
	}

	return true;
}

} /* namespace */
