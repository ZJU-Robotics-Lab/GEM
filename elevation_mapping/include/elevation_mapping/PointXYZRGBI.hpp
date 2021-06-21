#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <boost/shared_ptr.hpp>

struct PointXYZRGBI
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;

    float covariance;
    float intensity;

    PointXYZRGBI() {}
    PointXYZRGBI(const PointXYZRGBI& input)
    {
        this->x = input.x;
        this->y = input.y;
        this->z = input.z;
        this->rgb = input.rgb;
        this->intensity = input.intensity;
        this->covariance = input.covariance;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBI,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, intensity, intensity)
                                   (float, covariance, covariance)
);

PCL_INSTANTIATE(VoxelGrid, PointXYZRGBI);
PCL_INSTANTIATE(KdTree, PointXYZRGBI);
PCL_INSTANTIATE(KdTreeFLANN, PointXYZRGBI);

typedef pcl::PointCloud<PointXYZRGBI> PointCloudXYZRGBI;
