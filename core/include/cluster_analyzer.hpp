#ifndef CLUSTER_ANALYZER_HPP_
#define CLUSTER_ANALYZER_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "cluster_info.hpp"
#include "constants.hpp"

namespace brother_eye {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

/**
 * @brief Cluster analysis utilities
 * 
 * This class provides methods for extracting clusters from point clouds
 * and analyzing their risk levels based on distance and size.
 */
class ClusterAnalyzer {
 public:
  /**
   * @brief Extracts clusters from a point cloud and calculates bounding boxes
   * @param cloud Input point cloud
   * @param tolerance Clustering tolerance
   * @param min_size Minimum cluster size
   * @param max_size Maximum cluster size
   * @return Vector of cluster data with bounding boxes
   */
  static std::vector<ClusterData> ExtractClustersWithBoundingBoxes(
      const CloudT::Ptr& cloud,
      float tolerance = ClusteringConstants::kDefaultTolerance,
      int min_size = ClusteringConstants::kDefaultMinClusterSize,
      int max_size = ClusteringConstants::kDefaultMaxClusterSize);
  
  /**
   * @brief Analyzes cluster risk levels based on distance and size
   * @param cluster_data Vector of cluster data
   * @return Vector of cluster information with risk analysis
   */
  static std::vector<ClusterInfo> AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data);
  
 private:
  /**
   * @brief Calculates bounding box for a cluster
   * @param cloud Point cloud
   * @param indices Point indices belonging to the cluster
   * @param cluster_data Output cluster data to populate
   */
  static void CalculateClusterBoundingBox(const CloudT::Ptr& cloud, 
                                          const std::vector<int>& indices,
                                          ClusterData& cluster_data);
};

}  // namespace brother_eye

#endif  // CLUSTER_ANALYZER_HPP_
