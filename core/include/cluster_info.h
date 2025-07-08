#ifndef CLUSTER_INFO_H_
#define CLUSTER_INFO_H_

#include <opencv2/opencv.hpp>

namespace brother_eye {

enum class RiskLevel { 
  kNone = 0, 
  kGreen = 1, 
  kYellow = 2, 
  kRed = 3 
};

class ClusterInfo {
 public:
  ClusterInfo(const cv::Point2f& centroid, int cluster_id);
  
  // Getters
  cv::Point2f GetCentroid() const { return centroid_; }
  float GetDistance() const { return distance_; }
  float GetAngle() const { return angle_; }
  RiskLevel GetRisk() const { return risk_; }
  int GetId() const { return id_; }
  cv::Point3f GetBboxCenter() const { return bbox_center_; }
  cv::Point3f GetBboxSize() const { return bbox_size_; }
  
  // Setters
  void SetBboxCenter(const cv::Point3f& center) { bbox_center_ = center; }
  void SetBboxSize(const cv::Point3f& size) { bbox_size_ = size; }
  
 private:
  cv::Point2f centroid_;
  float distance_;
  float angle_;  // in radians
  RiskLevel risk_;
  int id_;
  
  // 3D Bounding Box data
  cv::Point3f bbox_center_;
  cv::Point3f bbox_size_;  // width, height, depth
  
  void CalculateRisk();
};

class ClusterData {
 public:
  ClusterData() = default;
  
  // Getters
  cv::Point2f GetCentroid() const { return centroid_; }
  cv::Point3f GetBboxCenter() const { return bbox_center_; }
  cv::Point3f GetBboxSize() const { return bbox_size_; }
  const std::vector<int>& GetPointIndices() const { return point_indices_; }
  
  // Setters
  void SetCentroid(const cv::Point2f& centroid) { centroid_ = centroid; }
  void SetBboxCenter(const cv::Point3f& center) { bbox_center_ = center; }
  void SetBboxSize(const cv::Point3f& size) { bbox_size_ = size; }
  void AddPointIndex(int index) { point_indices_.push_back(index); }
  
 private:
  cv::Point2f centroid_;
  cv::Point3f bbox_center_;
  cv::Point3f bbox_size_;
  std::vector<int> point_indices_;
};

}  // namespace brother_eye

#endif  // CLUSTER_INFO_H_
