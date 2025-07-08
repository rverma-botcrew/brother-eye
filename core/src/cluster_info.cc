#include "cluster_info.hpp"
#include <cmath>

namespace brother_eye {

ClusterInfo::ClusterInfo(const cv::Point2f& centroid, int cluster_id) 
    : centroid_(centroid), cluster_id_(cluster_id) {
  distance_ = std::hypot(centroid.x, centroid.y);
  angle_ = std::atan2(centroid.y, centroid.x);
  
  CalculateRisk();
  
  // Default bounding box (will be calculated later)
  bounding_box_center_ = cv::Point3f(centroid.x, centroid.y, DefaultBoundingBox::kDefaultHeight);
  bounding_box_size_ = cv::Point3f(DefaultBoundingBox::kDefaultWidth, 
                                   DefaultBoundingBox::kDefaultDepth, 
                                   DefaultBoundingBox::kDefaultTotalHeight);
}

void ClusterInfo::CalculateRisk() {
  if (distance_ < RiskThresholds::kRedRiskDistance) {
    risk_level_ = RiskLevel::kRed;
  } else if (distance_ < RiskThresholds::kYellowRiskDistance) {
    risk_level_ = RiskLevel::kYellow;
  } else if (distance_ < RiskThresholds::kGreenRiskDistance) {
    risk_level_ = RiskLevel::kGreen;
  } else {
    risk_level_ = RiskLevel::kNone;
  }
}

}  // namespace brother_eye
