#ifndef CLUSTER_VISUALIZER_HPP_
#define CLUSTER_VISUALIZER_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

#include "cluster_info.hpp"

namespace brother_eye {

/**
 * @brief Cluster visualization utilities
 * 
 * Provides methods for visualizing cluster risk analysis in a circular UI
 * with distance and angle information.
 */
class ClusterVisualizer {
 public:
  /**
   * @brief Configuration for visualization display
   */
  struct Config {
    int display_radius;
    int display_padding;
    int grid_spacing;
    int angle_step_degrees;
    float max_display_distance;
    float distance_scale;
    
    // Default constructor with default values
    Config() 
      : display_radius(200)
      , display_padding(10)
      , grid_spacing(50)
      , angle_step_degrees(30)
      , max_display_distance(10.0f)
      , distance_scale(20.0f) {}
  };
  
  /**
   * @brief Displays cluster risk analysis in a circular UI
   * @param clusters Vector of cluster information to display
   * @param config Visualization configuration
   */
  static void DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters, 
                                   const Config& config = Config());
  
 private:
  /**
   * @brief Draws the grid and reference lines for visualization
   * @param img Output image to draw on
   * @param center Center point of the display
   * @param radius Display radius
   * @param config Visualization configuration
   */
  static void DrawVisualizationGrid(cv::Mat& img, const cv::Point& center, 
                                   int radius, const Config& config);
  
  /**
   * @brief Draws a single cluster on the visualization
   * @param img Output image to draw on
   * @param cluster Cluster information
   * @param center Center point of the display
   * @param radius Display radius
   * @param config Visualization configuration
   */
  static void DrawCluster(cv::Mat& img, const ClusterInfo& cluster, 
                         const cv::Point& center, int radius, const Config& config);
  
  /**
   * @brief Gets color for risk level
   * @param risk_level Risk level to get color for
   * @return OpenCV color scalar
   */
  static cv::Scalar GetRiskColor(RiskLevel risk_level);
};

}  // namespace brother_eye

#endif  // CLUSTER_VISUALIZER_HPP_
