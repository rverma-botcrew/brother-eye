#ifndef POINTCLOUD_PROCESSOR_HPP_
#define POINTCLOUD_PROCESSOR_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <memory>
#include <set>

// Include modular filters and analyzers
#include "pointcloud_filters.hpp"
#include "cluster_analyzer.hpp"
#include "object_tracker.hpp"
#include "cluster_visualizer.hpp"
#include "data_converter.hpp"

#include "cluster_info.hpp"
#include "kalman_tracker.hpp"
#include "dds_pcl.hpp"

namespace brother_eye {

// Type aliases for cleaner code
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;

// Point cloud processing configuration
namespace ProcessingConfig {
  // Voxel grid filtering
  constexpr float kVoxelLeafSize = 0.05f;           ///< Voxel grid leaf size (5cm)
  
  // Z-axis filtering (height filtering)
  constexpr float kZFilterMin = -2.0f;              ///< Minimum Z value (2m below sensor)
  constexpr float kZFilterMax = 10.0f;              ///< Maximum Z value (10m above sensor)
  
  // Ground plane removal (RANSAC)
  constexpr float kRansacDistanceThreshold = 0.02f; ///< RANSAC distance threshold (2cm)
  constexpr int kRansacMaxIterations = 100;         ///< Maximum RANSAC iterations
  
  // Statistical outlier removal
  constexpr int kStatisticalOutlierMeanK = 20;      ///< Number of neighbors for outlier detection
  constexpr float kStatisticalOutlierStddevMul = 2.0f; ///< Standard deviation multiplier
  
  // Clustering parameters
  constexpr float kClusterTolerance = 0.2f;         ///< Euclidean clustering tolerance (20cm)
  constexpr int kMinClusterSize = 10;               ///< Minimum points per cluster
  constexpr int kMaxClusterSize = 5000;             ///< Maximum points per cluster
  
  // Tracking parameters
  constexpr int kMaxLostFrames = 5;                 ///< Max frames before object is dropped
  constexpr float kTrackingMatchThreshold = 0.3f;   ///< Distance threshold for object matching
}

// Visualization configuration
namespace VisualizationConfig {
  constexpr int kDisplayRadius = 200;               ///< Display radius in pixels
  constexpr int kDisplayPadding = 10;               ///< Padding around display
  constexpr int kGridSpacing = 50;                  ///< Grid line spacing
  constexpr int kAngleStepDegrees = 30;             ///< Angle grid step
  constexpr float kMaxDisplayDistance = 10.0f;      ///< Maximum display distance (meters)
  constexpr float kDistanceScale = 20.0f;           ///< Distance scaling factor
}

/**
 * @brief Main point cloud processing pipeline
 * 
 * This class orchestrates the complete point cloud processing workflow by
 * coordinating modular components for filtering, clustering, tracking, and visualization.
 * The actual processing is delegated to specialized modules for better maintainability.
 */
class PointCloudProcessor {
 public:
  /**
   * @brief Constructs a new PointCloudProcessor
   */
  PointCloudProcessor();
  
  /**
   * @brief Destructor
   */
  ~PointCloudProcessor() = default;
  
  // Delete copy constructor and assignment operator
  PointCloudProcessor(const PointCloudProcessor&) = delete;
  PointCloudProcessor& operator=(const PointCloudProcessor&) = delete;
  
  // ============================================================================
  // Main Processing Pipeline
  // ============================================================================
  
  /**
   * @brief Main processing pipeline for point cloud data
   * @param input_msg DDS point cloud message to process
   * @return Processed point cloud with tracked objects
   */
  CloudT::Ptr ProcessPointCloud(const DDSPointCloud2& input_msg);
  
  // ============================================================================
  // Individual Processing Steps (mostly delegated to modular components)
  // ============================================================================
  
  /**
   * @brief Converts DDS point cloud message to PCL format
   * @param msg DDS point cloud message
   * @return PCL point cloud
   */
  CloudT::Ptr ConvertToPcl(const DDSPointCloud2& msg);
  
  /**
   * @brief Applies filtering pipeline to clean the point cloud
   * @param input_cloud Raw point cloud to clean
   * @return Cleaned point cloud
   */
  CloudT::Ptr CleanCloud(const CloudT::Ptr& input_cloud);
  
  /**
   * @brief Extracts clusters and calculates their bounding boxes
   * @param cloud Cleaned point cloud
   * @return Vector of cluster data with bounding boxes
   */
  std::vector<ClusterData> ExtractClustersWithBoundingBoxes(const CloudT::Ptr& cloud);
  
  /**
   * @brief Analyzes cluster risk levels
   * @param cluster_data Vector of cluster data
   * @return Vector of cluster information with risk analysis
   */
  std::vector<ClusterInfo> AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data);
  
  /**
   * @brief Updates object tracking with new centroid positions
   * @param centroids Vector of cluster centroids
   */
  void UpdateTracking(const std::vector<cv::Point2f>& centroids);
  
  /**
   * @brief Removes old trackers that have been lost too long
   */
  void CleanupOldTrackers();
  
  /**
   * @brief Creates a point cloud representing tracked objects
   * @return Point cloud with tracked object positions
   */
  CloudT::Ptr CreateTrackedPointCloud();
  
  // ============================================================================
  // Visualization
  // ============================================================================
  
  /**
   * @brief Displays cluster risk analysis in a circular UI
   * @param clusters Vector of cluster information to display
   */
  void DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters);
  
  // ============================================================================
  // Accessors
  // ============================================================================
  
  /**
   * @brief Gets the currently tracked objects
   * @return Const reference to tracked objects map
   */
  const std::map<int, TrackedObject>& GetTrackedObjects() const noexcept;
  
  /**
   * @brief Gets the current frame count
   * @return Current frame number
   */
  size_t GetFrameCount() const noexcept { 
    return frame_count_; 
  }
  
  /**
   * @brief Increments the frame counter
   */
  void IncrementFrameCount() noexcept { 
    ++frame_count_; 
  }
  
 private:
  // ============================================================================
  // Member Variables
  // ============================================================================
  
  std::unique_ptr<ObjectTracker> object_tracker_;  ///< Object tracking component
  size_t frame_count_;                             ///< Current frame counter
};

}  // namespace brother_eye

#endif  // POINTCLOUD_PROCESSOR_HPP_
