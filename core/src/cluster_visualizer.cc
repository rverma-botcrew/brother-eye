#include "cluster_visualizer.hpp"
#include "constants.hpp"

#include <iostream>
#include <cmath>

namespace brother_eye {

void ClusterVisualizer::DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters, 
                                             const Config& config) {
  const int radius = config.display_radius;
  const int padding = config.display_padding;
  const cv::Point center(radius + padding, radius + padding);
  const int img_size = 2 * radius + 2 * padding;

  // Create dark background
  cv::Mat display_image(img_size, img_size, VisualizationConstants::kDisplayImageType, 
                       cv::Scalar(VisualizationConstants::kBackgroundColorB, 
                                 VisualizationConstants::kBackgroundColorG, 
                                 VisualizationConstants::kBackgroundColorR));

  // Draw grid and reference elements
  DrawVisualizationGrid(display_image, center, radius, config);

  // Draw all clusters
  for (const auto& cluster : clusters) {
    DrawCluster(display_image, cluster, center, radius, config);
  }

  // Draw center point (sensor position)
  cv::circle(display_image, center, VisualizationConstants::kCenterMarkerRadius, 
             cv::Scalar(VisualizationConstants::kCenterMarkerColorB, 
                       VisualizationConstants::kCenterMarkerColorG, 
                       VisualizationConstants::kCenterMarkerColorR), 
             VisualizationConstants::kCenterMarkerThickness);
  
  // Add title
  cv::putText(display_image, "Cluster Risk Analysis", 
              cv::Point(VisualizationConstants::kTextPositionX, VisualizationConstants::kTextPositionY), 
              cv::FONT_HERSHEY_SIMPLEX, VisualizationConstants::kTextFontScale, 
              cv::Scalar(VisualizationConstants::kCenterMarkerColorB, 
                        VisualizationConstants::kCenterMarkerColorG, 
                        VisualizationConstants::kCenterMarkerColorR), 
              VisualizationConstants::kTextThickness);

  // Add legend
  const int legend_y = img_size - 60;
  cv::putText(display_image, "Risk: ", cv::Point(10, legend_y), 
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  
  cv::circle(display_image, cv::Point(60, legend_y - 5), 5, GetRiskColor(RiskLevel::kRed), -1);
  cv::putText(display_image, "High", cv::Point(70, legend_y), 
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  
  cv::circle(display_image, cv::Point(120, legend_y - 5), 5, GetRiskColor(RiskLevel::kYellow), -1);
  cv::putText(display_image, "Med", cv::Point(130, legend_y), 
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  
  cv::circle(display_image, cv::Point(170, legend_y - 5), 5, GetRiskColor(RiskLevel::kGreen), -1);
  cv::putText(display_image, "Low", cv::Point(180, legend_y), 
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

  cv::imshow("Cluster Risk Analysis", display_image);
  cv::waitKey(1);
}

void ClusterVisualizer::DrawVisualizationGrid(cv::Mat& img, const cv::Point& center, 
                                             int radius, const Config& config) {
  const cv::Scalar grid_color(80, 80, 80);
  
  // Draw distance circles
  for (int r = config.grid_spacing; r <= radius; r += config.grid_spacing) {
    cv::circle(img, center, r, grid_color, 1);
  }
  
  // Draw angle lines
  for (int angle = 0; angle < 360; angle += config.angle_step_degrees) {
    const float rad = angle * M_PI / 180.0f;
    const cv::Point end_point(
      center.x + radius * std::cos(rad),
      center.y + radius * std::sin(rad)
    );
    cv::line(img, center, end_point, grid_color, 1);
  }
}

void ClusterVisualizer::DrawCluster(cv::Mat& img, const ClusterInfo& cluster, 
                                   const cv::Point& center, int radius, const Config& config) {
  // Get color based on risk level
  cv::Scalar color = GetRiskColor(cluster.GetRiskLevel());

  // Scale distance to fit in display
  const float display_distance = std::min(
    cluster.GetDistance() * config.distance_scale, 
    static_cast<float>(radius)
  );
  
  // Calculate cluster position on display
  const cv::Point cluster_position(
    center.x + display_distance * std::cos(cluster.GetAngle()),
    center.y + display_distance * std::sin(cluster.GetAngle())
  );

  // Draw cluster as filled circle with white border
  constexpr int kClusterRadius = 8;
  cv::circle(img, cluster_position, kClusterRadius, color, -1);
  cv::circle(img, cluster_position, kClusterRadius, cv::Scalar(255, 255, 255), 2);
  
  // Add cluster ID label
  const cv::Point text_position(cluster_position.x + 10, cluster_position.y - 5);
  cv::putText(img, std::to_string(cluster.GetClusterId()), 
              text_position, cv::FONT_HERSHEY_SIMPLEX, 0.4, 
              cv::Scalar(255, 255, 255), 1);
}

cv::Scalar ClusterVisualizer::GetRiskColor(RiskLevel risk_level) {
  switch (risk_level) {
    case RiskLevel::kRed:    return cv::Scalar(0, 0, 255);     // Red
    case RiskLevel::kYellow: return cv::Scalar(0, 255, 255);   // Yellow
    case RiskLevel::kGreen:  return cv::Scalar(0, 255, 0);     // Green
    default:                 return cv::Scalar(100, 100, 100); // Gray
  }
}

}  // namespace brother_eye
