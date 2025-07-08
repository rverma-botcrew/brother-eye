#ifndef POINTCLOUD_PROCESSOR_H_
#define POINTCLOUD_PROCESSOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <memory>
#include <set>

#include "cluster_info.h"
#include "kalman_tracker.h"
#include "dds_pcl.hpp"

namespace brother_eye {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;

class PointCloudProcessor {
 public:
  PointCloudProcessor();
  ~PointCloudProcessor() = default;
  
  // Main processing pipeline
  CloudT::Ptr ProcessPointCloud(const DDSPointCloud2& input_msg);
  
  // Individual processing steps
  CloudT::Ptr ConvertToPcl(const DDSPointCloud2& msg);
  CloudT::Ptr CleanCloud(const CloudT::Ptr& input_cloud);
  std::vector<ClusterData> ExtractClustersWithBoundingBoxes(const CloudT::Ptr& cloud);
  std::vector<ClusterInfo> AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data);
  
  // Tracking functionality
  void UpdateTracking(const std::vector<cv::Point2f>& centroids);
  void CleanupOldTrackers();
  CloudT::Ptr CreateTrackedPointCloud();
  
  // Visualization
  void DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters);
  
  // Getters
  const std::map<int, TrackedObject>& GetTrackedObjects() const { return tracked_objects_; }
  size_t GetFrameCount() const { return frame_count_; }
  
  void IncrementFrameCount() { ++frame_count_; }
  
 private:
  std::map<int, TrackedObject> tracked_objects_;
  int next_id_;
  size_t frame_count_;
  
  // Configuration constants
  static constexpr float kVoxelLeafSize = 0.05f;
  static constexpr float kZFilterMin = -2.0f;
  static constexpr float kZFilterMax = 10.0f;
  static constexpr float kRansacDistanceThreshold = 0.02f;
  static constexpr int kRansacMaxIterations = 100;
  static constexpr int kStatisticalOutlierMeanK = 20;
  static constexpr float kStatisticalOutlierStddevMul = 2.0f;
  static constexpr float kClusterTolerance = 0.2f;
  static constexpr int kMinClusterSize = 10;
  static constexpr int kMaxClusterSize = 5000;
  static constexpr int kMaxLostFrames = 5;
  static constexpr float kTrackingMatchThreshold = 0.3f;
};

}  // namespace brother_eye

#endif  // POINTCLOUD_PROCESSOR_H_
