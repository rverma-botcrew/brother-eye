#ifndef CLUSTER_INFO_HPP_
#define CLUSTER_INFO_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include "constants.hpp"

namespace brother_eye {

/**
 * @brief Enumeration representing different risk levels for detected objects
 * 
 * Risk levels are used to categorize the potential threat level of detected
 * objects in the point cloud processing pipeline.
 */
enum class RiskLevel { 
  kNone = 0,    ///< No risk detected
  kGreen = 1,   ///< Low risk - safe distance
  kYellow = 2,  ///< Medium risk - caution required
  kRed = 3      ///< High risk - immediate attention required
};

/**
 * @brief Represents information about a detected cluster/object in the point cloud
 * 
 * This class encapsulates all relevant information about a detected object,
 * including its position, risk assessment, and 3D bounding box data.
 */
class ClusterInfo {
 public:
  /**
   * @brief Constructs a ClusterInfo object with centroid and ID
   * @param centroid The 2D centroid position of the cluster
   * @param cluster_id Unique identifier for this cluster
   */
  ClusterInfo(const cv::Point2f& centroid, int cluster_id);
  
  // Getters
  cv::Point2f GetCentroid() const noexcept { return centroid_; }
  float GetDistance() const noexcept { return distance_; }
  float GetAngle() const noexcept { return angle_; }
  RiskLevel GetRiskLevel() const noexcept { return risk_level_; }
  int GetClusterId() const noexcept { return cluster_id_; }
  cv::Point3f GetBoundingBoxCenter() const noexcept { return bounding_box_center_; }
  cv::Point3f GetBoundingBoxSize() const noexcept { return bounding_box_size_; }
  
  // Setters
  void SetBoundingBoxCenter(const cv::Point3f& center) { bounding_box_center_ = center; }
  void SetBoundingBoxSize(const cv::Point3f& size) { bounding_box_size_ = size; }
  
 private:
  // 2D position data
  cv::Point2f centroid_;
  float distance_;
  float angle_;  ///< Angle in radians relative to sensor
  
  // Classification data
  RiskLevel risk_level_;
  int cluster_id_;
  
  // 3D bounding box data
  cv::Point3f bounding_box_center_;
  cv::Point3f bounding_box_size_;  ///< Width, height, depth
  
  /**
   * @brief Calculates the risk level based on distance and other factors
   */
  void CalculateRisk();
};

/**
 * @brief Container class for raw cluster data during processing
 * 
 * This class holds intermediate data about clusters during the point cloud
 * processing pipeline, before they are converted to ClusterInfo objects.
 */
class ClusterData {
 public:
  ClusterData() = default;
  
  // Getters
  cv::Point2f GetCentroid() const noexcept { return centroid_; }
  cv::Point3f GetBoundingBoxCenter() const noexcept { return bounding_box_center_; }
  cv::Point3f GetBoundingBoxSize() const noexcept { return bounding_box_size_; }
  const std::vector<int>& GetPointIndices() const noexcept { return point_indices_; }
  
  // Setters
  void SetCentroid(const cv::Point2f& centroid) { centroid_ = centroid; }
  void SetBoundingBoxCenter(const cv::Point3f& center) { bounding_box_center_ = center; }
  void SetBoundingBoxSize(const cv::Point3f& size) { bounding_box_size_ = size; }
  void AddPointIndex(int index) { point_indices_.push_back(index); }
  
 private:
  // 2D position data
  cv::Point2f centroid_;
  
  // 3D bounding box data
  cv::Point3f bounding_box_center_;
  cv::Point3f bounding_box_size_;  ///< Width, height, depth
  
  // Point cloud indices
  std::vector<int> point_indices_;  ///< Indices of points belonging to this cluster
};

}  // namespace brother_eye

#endif  // CLUSTER_INFO_HPP_
