#include "pointcloud_processor.hpp"
#include "constants.hpp"

#include <iostream>
#include <algorithm>

namespace brother_eye {

// ============================================================================
// Constructor
// ============================================================================

PointCloudProcessor::PointCloudProcessor() : frame_count_(SystemConstants::kInitialFrameCount) {
  // Initialize modular components
  object_tracker_ = std::make_unique<ObjectTracker>();
}

// ============================================================================
// Main Processing Pipeline
// ============================================================================

CloudT::Ptr PointCloudProcessor::ProcessPointCloud(const DDSPointCloud2& input_msg) {
  IncrementFrameCount();
  
  // Convert and clean the point cloud using modular components
  auto cloud_raw = DataConverter::ConvertToPcl(input_msg);
  auto cleaned = PointCloudFilters::CleanCloud(cloud_raw);
  auto cluster_data = ClusterAnalyzer::ExtractClustersWithBoundingBoxes(cleaned);
  
  std::cout << "[PROCESSOR] 📊 Found " << cluster_data.size() << " clusters\n";
  
  // Extract centroids for tracking
  std::vector<cv::Point2f> centroids;
  centroids.reserve(cluster_data.size());
  for (const auto& data : cluster_data) {
    centroids.push_back(data.GetCentroid());
  }
  
  // Update tracking system using modular tracker
  object_tracker_->UpdateTracking(centroids);
  object_tracker_->CleanupOldTrackers();
  
  return object_tracker_->CreateTrackedPointCloud();
}

// ============================================================================
// Data Conversion (delegated to DataConverter)
// ============================================================================

CloudT::Ptr PointCloudProcessor::ConvertToPcl(const DDSPointCloud2& msg) {
  return DataConverter::ConvertToPcl(msg);
}

// ============================================================================
// Point Cloud Cleaning Pipeline (delegated to PointCloudFilters)
// ============================================================================

CloudT::Ptr PointCloudProcessor::CleanCloud(const CloudT::Ptr& input_cloud) {
  return PointCloudFilters::CleanCloud(input_cloud);
}

// ============================================================================
// Cluster Extraction and Analysis (delegated to ClusterAnalyzer)
// ============================================================================

std::vector<ClusterData> PointCloudProcessor::ExtractClustersWithBoundingBoxes(const CloudT::Ptr& cloud) {
  // Use more permissive clustering parameters
  return ClusterAnalyzer::ExtractClustersWithBoundingBoxes(cloud, 
                                                           ClusteringConstants::kDefaultTolerance, 
                                                           ClusteringConstants::kDefaultMinClusterSize, 
                                                           ClusteringConstants::kDefaultMaxClusterSize);
}

std::vector<ClusterInfo> PointCloudProcessor::AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data) {
  return ClusterAnalyzer::AnalyzeClusterRisk(cluster_data);
}

// ============================================================================
// Object Tracking (delegated to ObjectTracker)
// ============================================================================

void PointCloudProcessor::UpdateTracking(const std::vector<cv::Point2f>& centroids) {
  object_tracker_->UpdateTracking(centroids);
}

void PointCloudProcessor::CleanupOldTrackers() {
  object_tracker_->CleanupOldTrackers();
}

CloudT::Ptr PointCloudProcessor::CreateTrackedPointCloud() {
  return object_tracker_->CreateTrackedPointCloud();
}

const std::map<int, TrackedObject>& PointCloudProcessor::GetTrackedObjects() const noexcept {
  return object_tracker_->GetTrackedObjects();
}

// ============================================================================
// Visualization (delegated to ClusterVisualizer)
// ============================================================================

void PointCloudProcessor::DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters) {
  ClusterVisualizer::DisplayClusterRiskUi(clusters);
}

}  // namespace brother_eye
