#include "cluster_info.h"
#include <cmath>

namespace brother_eye {

ClusterInfo::ClusterInfo(const cv::Point2f& centroid, int cluster_id) 
    : centroid_(centroid), id_(cluster_id) {
  distance_ = std::hypot(centroid.x, centroid.y);
  angle_ = std::atan2(centroid.y, centroid.x);
  
  CalculateRisk();
  
  // Default bounding box (will be calculated later)
  bbox_center_ = cv::Point3f(centroid.x, centroid.y, 0.5f);  // Assume 0.5m height center
  bbox_size_ = cv::Point3f(0.5f, 0.5f, 1.0f);  // Default size: 0.5x0.5x1.0m
}

void ClusterInfo::CalculateRisk() {
  if (distance_ < 1.5f) {
    risk_ = RiskLevel::kRed;
  } else if (distance_ < 2.0f) {
    risk_ = RiskLevel::kYellow;
  } else if (distance_ < 5.0f) {
    risk_ = RiskLevel::kGreen;
  } else {
    risk_ = RiskLevel::kNone;
  }
}

}  // namespace brother_eye
