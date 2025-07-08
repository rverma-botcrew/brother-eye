#include "pointcloud_filters.hpp"
#include "constants.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

namespace brother_eye {
namespace PointCloudFilters {

CloudT::Ptr ApplyVoxelFiltering(const CloudT::Ptr& cloud, float leaf_size) {
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  
  CloudT::Ptr filtered(new CloudT);
  voxel_grid.filter(*filtered);
  return filtered;
}

CloudT::Ptr ApplyRangeFiltering(const CloudT::Ptr& cloud, float z_min, float z_max) {
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud(cloud);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(z_min, z_max);
  
  CloudT::Ptr filtered(new CloudT);
  pass_through.filter(*filtered);
  return filtered;
}

CloudT::Ptr RemoveGroundPlane(const CloudT::Ptr& cloud, 
                                                 float distance_threshold, 
                                                 int max_iterations) {
  // Skip ground removal if cloud is too small for RANSAC
  if (cloud->points.size() < FilteringConstants::kMinPointsForRobustProcessing) {
    std::cout << "[FILTERS] ⚠️ Skipping ground removal: too few points (" << cloud->points.size() << ")\n";
    return cloud;
  }

  // Configure RANSAC plane segmentation
  pcl::SACSegmentation<PointT> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(max_iterations);
  segmentation.setDistanceThreshold(distance_threshold);
  segmentation.setInputCloud(cloud);

  // Find ground plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  segmentation.segment(*inliers, *coefficients);

  // Check if segmentation was successful
  if (inliers->indices.empty()) {
    std::cout << "[FILTERS] ⚠️ No ground plane found, keeping all points\n";
    return cloud;
  }

  // Extract non-ground points
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // Extract everything except the ground plane
  
  CloudT::Ptr no_ground(new CloudT);
  extract.filter(*no_ground);
  
  // If ground removal removed too many points, return original cloud
  if (no_ground->points.size() < cloud->points.size() * FilteringConstants::kMinGroundRetentionRatio) {
    std::cout << "[FILTERS] ⚠️ Ground removal too aggressive, keeping original cloud\n";
    return cloud;
  }
  
  return no_ground;
}

CloudT::Ptr RemoveStatisticalOutliers(const CloudT::Ptr& cloud, 
                                                         int mean_k, 
                                                         float stddev_mul) {
  // Skip outlier removal if cloud is too small
  if (cloud->points.size() < mean_k * FilteringConstants::kOutlierMeanKMultiplier) {
    std::cout << "[FILTERS] ⚠️ Skipping outlier removal: too few points (" << cloud->points.size() << ")\n";
    return cloud;
  }

  pcl::StatisticalOutlierRemoval<PointT> outlier_removal;
  outlier_removal.setInputCloud(cloud);
  outlier_removal.setMeanK(mean_k);
  outlier_removal.setStddevMulThresh(stddev_mul);
  
  CloudT::Ptr cleaned(new CloudT);
  outlier_removal.filter(*cleaned);
  
  // If outlier removal removed too many points, return original cloud
  if (cleaned->points.size() < cloud->points.size() * FilteringConstants::kMinOutlierRetentionRatio) {
    std::cout << "[FILTERS] ⚠️ Outlier removal too aggressive, keeping original cloud\n";
    return cloud;
  }
  
  return cleaned;
}

CloudT::Ptr CleanCloud(const CloudT::Ptr& input_cloud) {
  std::cout << "[FILTERS] 🔧 Starting with " << input_cloud->points.size() << " points\n";
  
  if (input_cloud->points.size() < FilteringConstants::kMinPointsForFiltering) {
    std::cout << "[FILTERS] ⚠️ Too few points for aggressive filtering, using minimal processing\n";
    // Skip most filtering for very small clouds
    auto range_filtered = ApplyRangeFiltering(input_cloud, FilteringConstants::kWideRangeMin, FilteringConstants::kWideRangeMax);  // Very wide range
    std::cout << "[FILTERS] 📊 After minimal filtering: " << range_filtered->points.size() << " points\n";
    return range_filtered;
  }
  
  // Use more relaxed parameters for filtering
  auto voxel_filtered = ApplyVoxelFiltering(input_cloud, FilteringConstants::kLargeVoxelLeafSize);  // Even larger voxel size
  std::cout << "[FILTERS] 📊 After voxel filtering: " << voxel_filtered->points.size() << " points\n";
  
  auto range_filtered = ApplyRangeFiltering(voxel_filtered, FilteringConstants::kWideRangeMin, FilteringConstants::kWideRangeMax);  // Very wide range
  std::cout << "[FILTERS] 📊 After range filtering: " << range_filtered->points.size() << " points\n";
  
  // Skip ground plane removal if too few points
  if (range_filtered->points.size() < FilteringConstants::kMinPointsForVoxelOnly) {
    std::cout << "[FILTERS] ⚠️ Skipping ground plane removal - too few points\n";
    return range_filtered;
  }
  
  auto no_ground = RemoveGroundPlane(range_filtered, FilteringConstants::kRelaxedGroundThreshold, FilteringConstants::kRelaxedGroundIterations);  // Very relaxed ground removal
  std::cout << "[FILTERS] 📊 After ground removal: " << no_ground->points.size() << " points\n";
  
  // Skip outlier removal if too few points
  if (no_ground->points.size() < FilteringConstants::kMinPointsForGroundRemoval) {
    std::cout << "[FILTERS] ⚠️ Skipping outlier removal - too few points\n";
    return no_ground;
  }
  
  auto cleaned = RemoveStatisticalOutliers(no_ground, FilteringConstants::kRelaxedOutlierMeanK, FilteringConstants::kRelaxedOutlierStdDev);  // Very relaxed outlier removal
  std::cout << "[FILTERS] 📊 After outlier removal: " << cleaned->points.size() << " points\n";
  
  return cleaned;
}

}  // namespace PointCloudFilters
}  // namespace brother_eye
