#pragma once

// Standard library
#include <memory>

// PCL core types
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Project includes
#include "constants.hpp"

// Note: filter-specific headers are included in the implementation (.cpp) file

namespace brother_eye {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

/**
 * @brief Point cloud filtering utilities
 * 
 * This class provides static methods for various point cloud filtering operations
 * including voxel filtering, range filtering, ground plane removal, and outlier removal.
 */
namespace PointCloudFilters {

/**
 * @brief Applies voxel grid filtering to reduce point density
 * @param cloud Input point cloud
 * @param leaf_size Voxel grid leaf size
 * @return Voxel-filtered point cloud
 */
CloudT::Ptr ApplyVoxelFiltering(const CloudT::Ptr& cloud, float leaf_size = FilteringConstants::kDefaultVoxelLeafSize);

/**
 * @brief Applies pass-through filtering to remove points outside Z range
 * @param cloud Input point cloud
 * @param z_min Minimum Z value
 * @param z_max Maximum Z value
 * @return Range-filtered point cloud
 */
CloudT::Ptr ApplyRangeFiltering(const CloudT::Ptr& cloud, float z_min = FilteringConstants::kDefaultRangeMin, float z_max = FilteringConstants::kDefaultRangeMax);

/**
 * @brief Removes ground plane using RANSAC plane segmentation
 * @param cloud Input point cloud
 * @param distance_threshold RANSAC distance threshold
 * @param max_iterations Maximum RANSAC iterations
 * @return Point cloud with ground plane removed
 */
CloudT::Ptr RemoveGroundPlane(const CloudT::Ptr& cloud, 
                                       float distance_threshold = FilteringConstants::kDefaultGroundThreshold, 
                                       int max_iterations = FilteringConstants::kDefaultGroundIterations);

/**
 * @brief Removes statistical outliers from point cloud
 * @param cloud Input point cloud
 * @param mean_k Number of neighbors for outlier detection
 * @param stddev_mul Standard deviation multiplier
 * @return Point cloud with outliers removed
 */
CloudT::Ptr RemoveStatisticalOutliers(const CloudT::Ptr& cloud, 
                                               int mean_k = FilteringConstants::kDefaultOutlierMeanK, 
                                               float stddev_mul = FilteringConstants::kDefaultOutlierStdDev);

/**
 * @brief Applies complete cleaning pipeline to point cloud
 * @param input_cloud Raw point cloud to clean
 * @return Cleaned point cloud
 */
CloudT::Ptr CleanCloud(const CloudT::Ptr& input_cloud);
}  // namespace PointCloudFilters

}  // namespace brother_eye
