#include "cluster_analyzer.hpp"
#include "constants.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <iomanip>

namespace brother_eye {

std::vector<ClusterData> ClusterAnalyzer::ExtractClustersWithBoundingBoxes(
    const CloudT::Ptr& cloud,
    float tolerance,
    int min_size,
    int max_size) {
  
  if (cloud->points.empty()) {
    std::cout << "[CLUSTER_ANALYZER] ⚠️ Empty cloud, no clusters extracted\n";
    return {};
  }

  // Create KD-tree for efficient neighbor search
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  // Extract clusters using Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::cout << "[CLUSTER_ANALYZER] 🔍 Extracted " << cluster_indices.size() << " clusters\n";

  // Calculate bounding boxes for each cluster
  std::vector<ClusterData> cluster_data;
  cluster_data.reserve(cluster_indices.size());
  
  for (const auto& indices : cluster_indices) {
    ClusterData data;
    CalculateClusterBoundingBox(cloud, indices.indices, data);
    cluster_data.push_back(data);
  }

  return cluster_data;
}

void ClusterAnalyzer::CalculateClusterBoundingBox(const CloudT::Ptr& cloud, 
                                                  const std::vector<int>& indices,
                                                  ClusterData& cluster_data) {
  if (indices.empty()) {
    return;
  }

  // Initialize bounds
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  
  // Calculate bounds and centroid
  for (int idx : indices) {
    const PointT& point = cloud->points[idx];
    
    // Accumulate for centroid calculation
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
    
    // Update bounds
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
    
    cluster_data.AddPointIndex(idx);
  }
  
  // Calculate centroid
  const int num_points = indices.size();
  cluster_data.SetCentroid(cv::Point2f(sum_x / num_points, sum_y / num_points));
  
  // Calculate 3D bounding box center
  cluster_data.SetBoundingBoxCenter(cv::Point3f(
    (min_x + max_x) / ClusteringConstants::kCenterCalculationDivisor,
    (min_y + max_y) / ClusteringConstants::kCenterCalculationDivisor,
    (min_z + max_z) / ClusteringConstants::kCenterCalculationDivisor
  ));
  
  // Calculate 3D bounding box size (with minimum dimensions)
  cluster_data.SetBoundingBoxSize(cv::Point3f(
    std::max(ClusteringConstants::kMinBoundingBoxDimension, max_x - min_x),
    std::max(ClusteringConstants::kMinBoundingBoxDimension, max_y - min_y),
    std::max(ClusteringConstants::kMinBoundingBoxDimension, max_z - min_z)
  ));
}

std::vector<ClusterInfo> ClusterAnalyzer::AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data) {
  std::vector<ClusterInfo> cluster_info;
  cluster_info.reserve(cluster_data.size());
  
  for (size_t i = 0; i < cluster_data.size(); ++i) {
    ClusterInfo info(cluster_data[i].GetCentroid(), static_cast<int>(i));
    
    // Set bounding box information
    info.SetBoundingBoxCenter(cluster_data[i].GetBoundingBoxCenter());
    info.SetBoundingBoxSize(cluster_data[i].GetBoundingBoxSize());
    
    cluster_info.push_back(info);
    
    // Print detailed cluster analysis
    const auto& bbox_size = info.GetBoundingBoxSize();
    std::cout << "[CLUSTER_ANALYZER] 📊 Cluster " << info.GetClusterId() 
              << " | Dist: " << std::fixed << std::setprecision(ClusteringConstants::kDistancePrecision) << info.GetDistance() << "m"
              << " | Angle: " << std::setprecision(ClusteringConstants::kAnglePrecision) << (info.GetAngle() * ClusteringConstants::kRadiansToDegrees) << "°"
              << " | Size: [" << std::setprecision(ClusteringConstants::kSizePrecision) 
              << bbox_size.x << "×" << bbox_size.y << "×" << bbox_size.z << "]m"
              << " | Risk: ";
    
    // Print risk level with appropriate emoji
    switch (info.GetRiskLevel()) {
      case RiskLevel::kRed:    std::cout << "🔴 RED";    break;
      case RiskLevel::kYellow: std::cout << "🟡 YELLOW"; break;
      case RiskLevel::kGreen:  std::cout << "🟢 GREEN";  break;
      default:                 std::cout << "⚪ NONE";   break;
    }
    std::cout << std::endl;
  }
  
  return cluster_info;
}

}  // namespace brother_eye
