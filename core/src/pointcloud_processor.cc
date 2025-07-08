#include "pointcloud_processor.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <set>
#include <iomanip>

namespace brother_eye {

// ============================================================================
// Constructor
// ============================================================================

PointCloudProcessor::PointCloudProcessor() : next_id_(0), frame_count_(0) {
}

// ============================================================================
// Main Processing Pipeline
// ============================================================================

CloudT::Ptr PointCloudProcessor::ProcessPointCloud(const DDSPointCloud2& input_msg) {
  IncrementFrameCount();
  
  // Convert and clean the point cloud
  auto cloud_raw = ConvertToPcl(input_msg);
  auto cleaned = CleanCloud(cloud_raw);
  auto cluster_data = ExtractClustersWithBoundingBoxes(cleaned);
  
  // Extract centroids for tracking
  std::vector<cv::Point2f> centroids;
  centroids.reserve(cluster_data.size());
  for (const auto& data : cluster_data) {
    centroids.push_back(data.GetCentroid());
  }
  
  // Update tracking system
  UpdateTracking(centroids);
  CleanupOldTrackers();
  
  return CreateTrackedPointCloud();
}

// ============================================================================
// Data Conversion
// ============================================================================

CloudT::Ptr PointCloudProcessor::ConvertToPcl(const DDSPointCloud2& msg) {
  const size_t point_count = msg.data().size() / msg.point_step();
  CloudT::Ptr cloud(new CloudT);
  
  // Initialize cloud properties
  cloud->width = static_cast<uint32_t>(point_count);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(point_count);

  // Convert binary data to PCL points
  const uint8_t* raw_data = msg.data().data();
  for (size_t i = 0; i < point_count; ++i) {
    const float* point_data = reinterpret_cast<const float*>(raw_data + i * msg.point_step());
    auto& point = cloud->points[i];
    point.x = point_data[0];
    point.y = point_data[1];
    point.z = point_data[2];
    point.intensity = point_data[3];
  }
  
  return cloud;
}

// ============================================================================
// Point Cloud Cleaning Pipeline
// ============================================================================

CloudT::Ptr PointCloudProcessor::CleanCloud(const CloudT::Ptr& input_cloud) {
  auto voxel_filtered = ApplyVoxelFiltering(input_cloud);
  auto range_filtered = ApplyRangeFiltering(voxel_filtered);
  auto no_ground = RemoveGroundPlane(range_filtered);
  auto cleaned = RemoveStatisticalOutliers(no_ground);
  
  return cleaned;
}

CloudT::Ptr PointCloudProcessor::ApplyVoxelFiltering(const CloudT::Ptr& cloud) {
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(ProcessingConfig::kVoxelLeafSize, 
                         ProcessingConfig::kVoxelLeafSize, 
                         ProcessingConfig::kVoxelLeafSize);
  
  CloudT::Ptr filtered(new CloudT);
  voxel_grid.filter(*filtered);
  return filtered;
}

CloudT::Ptr PointCloudProcessor::ApplyRangeFiltering(const CloudT::Ptr& cloud) {
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud(cloud);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(ProcessingConfig::kZFilterMin, ProcessingConfig::kZFilterMax);
  
  CloudT::Ptr filtered(new CloudT);
  pass_through.filter(*filtered);
  return filtered;
}

CloudT::Ptr PointCloudProcessor::RemoveGroundPlane(const CloudT::Ptr& cloud) {
  // Configure RANSAC plane segmentation
  pcl::SACSegmentation<PointT> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(ProcessingConfig::kRansacMaxIterations);
  segmentation.setDistanceThreshold(ProcessingConfig::kRansacDistanceThreshold);
  segmentation.setInputCloud(cloud);

  // Find ground plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  segmentation.segment(*inliers, *coefficients);

  // Extract non-ground points
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // Extract everything except the ground plane
  
  CloudT::Ptr no_ground(new CloudT);
  extract.filter(*no_ground);
  return no_ground;
}

CloudT::Ptr PointCloudProcessor::RemoveStatisticalOutliers(const CloudT::Ptr& cloud) {
  pcl::StatisticalOutlierRemoval<PointT> outlier_removal;
  outlier_removal.setInputCloud(cloud);
  outlier_removal.setMeanK(ProcessingConfig::kStatisticalOutlierMeanK);
  outlier_removal.setStddevMulThresh(ProcessingConfig::kStatisticalOutlierStddevMul);
  
  CloudT::Ptr cleaned(new CloudT);
  outlier_removal.filter(*cleaned);
  return cleaned;
}

// ============================================================================
// Cluster Extraction and Analysis
// ============================================================================

std::vector<ClusterData> PointCloudProcessor::ExtractClustersWithBoundingBoxes(const CloudT::Ptr& cloud) {
  // Set up KD-tree for efficient neighbor search
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  // Configure clustering algorithm
  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster;
  euclidean_cluster.setClusterTolerance(ProcessingConfig::kClusterTolerance);
  euclidean_cluster.setMinClusterSize(ProcessingConfig::kMinClusterSize);
  euclidean_cluster.setMaxClusterSize(ProcessingConfig::kMaxClusterSize);
  euclidean_cluster.setSearchMethod(tree);
  euclidean_cluster.setInputCloud(cloud);

  // Extract clusters
  std::vector<pcl::PointIndices> cluster_indices;
  euclidean_cluster.extract(cluster_indices);

  // Process each cluster
  std::vector<ClusterData> clusters;
  clusters.reserve(cluster_indices.size());
  
  for (const auto& indices : cluster_indices) {
    ClusterData cluster_data;
    CalculateClusterBoundingBox(cloud, indices.indices, cluster_data);
    clusters.push_back(cluster_data);
  }

  return clusters;
}

void PointCloudProcessor::CalculateClusterBoundingBox(const CloudT::Ptr& cloud, 
                                                      const std::vector<int>& indices,
                                                      ClusterData& cluster_data) {
  // Initialize bounds
  float sum_x = 0, sum_y = 0, sum_z = 0;
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  
  // Process each point in the cluster
  for (int idx : indices) {
    const PointT& point = cloud->points[idx];
    
    // Accumulate for centroid calculation
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
    
    // Update bounds
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
    
    cluster_data.AddPointIndex(idx);
  }
  
  // Calculate centroid
  const int num_points = indices.size();
  cluster_data.SetCentroid(cv::Point2f(sum_x / num_points, sum_y / num_points));
  
  // Calculate 3D bounding box center
  cluster_data.SetBoundingBoxCenter(cv::Point3f(
    (min_x + max_x) / 2.0f,
    (min_y + max_y) / 2.0f,
    (min_z + max_z) / 2.0f
  ));
  
  // Calculate 3D bounding box size (with minimum dimensions)
  constexpr float kMinDimension = 0.1f;  // 10cm minimum
  cluster_data.SetBoundingBoxSize(cv::Point3f(
    std::max(kMinDimension, max_x - min_x),
    std::max(kMinDimension, max_y - min_y),
    std::max(kMinDimension, max_z - min_z)
  ));
}

std::vector<ClusterInfo> PointCloudProcessor::AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data) {
  std::vector<ClusterInfo> cluster_info;
  cluster_info.reserve(cluster_data.size());
  
  for (size_t i = 0; i < cluster_data.size(); ++i) {
    ClusterInfo info(cluster_data[i].GetCentroid(), static_cast<int>(i));
    
    // Set bounding box information
    info.SetBoundingBoxCenter(cluster_data[i].GetBoundingBoxCenter());
    info.SetBoundingBoxSize(cluster_data[i].GetBoundingBoxSize());
    
    cluster_info.push_back(info);
    
    // Print detailed cluster analysis
    const auto& bbox_size = info.GetBoundingBoxSize();
    std::cout << "[PROCESSOR] 📊 Cluster " << info.GetClusterId() 
              << " | Dist: " << std::fixed << std::setprecision(2) << info.GetDistance() << "m"
              << " | Angle: " << std::setprecision(1) << (info.GetAngle() * 180.0f / M_PI) << "°"
              << " | Size: [" << std::setprecision(2) 
              << bbox_size.x << "×" << bbox_size.y << "×" << bbox_size.z << "]m"
              << " | Risk: ";
    
    // Print risk level with appropriate emoji
    switch (info.GetRiskLevel()) {
      case RiskLevel::kRed:    std::cout << "🔴 RED";    break;
      case RiskLevel::kYellow: std::cout << "🟡 YELLOW"; break;
      case RiskLevel::kGreen:  std::cout << "🟢 GREEN";  break;
      default:                 std::cout << "⚪ NONE";   break;
    }
    std::cout << std::endl;
  }
  
  return cluster_info;
}

// ============================================================================
// Object Tracking
// ============================================================================

void PointCloudProcessor::UpdateTracking(const std::vector<cv::Point2f>& centroids) {
  // Predict positions for all existing trackers
  for (auto& [id, tracker] : tracked_objects_) {
    tracker.Predict();
  }

  // Perform nearest neighbor matching with adaptive threshold
  std::set<int> matched_tracker_ids;
  
  for (const auto& centroid : centroids) {
    int best_tracker_id = -1;
    float best_distance = CalculateAdaptiveThreshold();
    
    // Find the closest unmatched tracker
    for (auto& [tracker_id, tracker] : tracked_objects_) {
      if (matched_tracker_ids.count(tracker_id) > 0) continue;
      
      const float distance = tracker.DistanceTo(centroid);
      if (distance < best_distance) {
        best_distance = distance;
        best_tracker_id = tracker_id;
      }
    }

    if (best_tracker_id == -1) {
      // Create new tracker for unmatched detection
      tracked_objects_[next_id_] = TrackedObject(centroid);
      matched_tracker_ids.insert(next_id_);
      ++next_id_;
    } else {
      // Update existing tracker
      tracked_objects_[best_tracker_id].Update(centroid);
      matched_tracker_ids.insert(best_tracker_id);
    }
  }
}

float PointCloudProcessor::CalculateAdaptiveThreshold() const {
  // Adaptive threshold based on number of tracked objects
  constexpr float kBaseThreshold = 0.4f;
  constexpr float kAdaptiveFactor = 0.05f;
  return std::max(kBaseThreshold, 1.0f - kAdaptiveFactor * tracked_objects_.size());
}

void PointCloudProcessor::CleanupOldTrackers() {
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if (it->second.GetLostFrames() > ProcessingConfig::kMaxLostFrames) {
      it = tracked_objects_.erase(it);
    } else {
      ++it;
    }
  }
}

CloudT::Ptr PointCloudProcessor::CreateTrackedPointCloud() {
  CloudT::Ptr tracked_cloud(new CloudT);
  tracked_cloud->points.reserve(tracked_objects_.size());
  
  for (const auto& [id, tracker] : tracked_objects_) {
    PointT point;
    point.x = tracker.GetLastCentroid().x;
    point.y = tracker.GetLastCentroid().y;
    point.z = 0.0f;
    point.intensity = 200.0f + (id % 55);  // Unique intensity per ID
    tracked_cloud->points.push_back(point);
  }
  
  tracked_cloud->width = tracked_cloud->points.size();
  tracked_cloud->height = 1;
  tracked_cloud->is_dense = true;
  
  return tracked_cloud;
}

// ============================================================================
// Visualization
// ============================================================================

void PointCloudProcessor::DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters) {
  const int radius = VisualizationConfig::kDisplayRadius;
  const int padding = VisualizationConfig::kDisplayPadding;
  const cv::Point center(radius + padding, radius + padding);
  const int img_size = 2 * radius + 2 * padding;

  // Create dark background
  cv::Mat display_image(img_size, img_size, CV_8UC3, cv::Scalar(30, 30, 30));

  // Draw grid and reference elements
  DrawVisualizationGrid(display_image, center, radius);

  // Draw all clusters
  for (const auto& cluster : clusters) {
    DrawCluster(display_image, cluster, center, radius);
  }

  // Draw center point (sensor position)
  cv::circle(display_image, center, 3, cv::Scalar(255, 255, 255), -1);
  
  // Add title
  cv::putText(display_image, "Cluster Risk Analysis", cv::Point(10, 20), 
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

  cv::imshow("Cluster Risk Analysis", display_image);
  cv::waitKey(1);
}

void PointCloudProcessor::DrawVisualizationGrid(cv::Mat& img, const cv::Point& center, int radius) {
  const cv::Scalar grid_color(80, 80, 80);
  
  // Draw distance circles
  for (int r = VisualizationConfig::kGridSpacing; r <= radius; r += VisualizationConfig::kGridSpacing) {
    cv::circle(img, center, r, grid_color, 1);
  }
  
  // Draw angle lines
  for (int angle = 0; angle < 360; angle += VisualizationConfig::kAngleStepDegrees) {
    const float rad = angle * M_PI / 180.0f;
    const cv::Point end_point(
      center.x + radius * std::cos(rad),
      center.y + radius * std::sin(rad)
    );
    cv::line(img, center, end_point, grid_color, 1);
  }
}

void PointCloudProcessor::DrawCluster(cv::Mat& img, const ClusterInfo& cluster, 
                                      const cv::Point& center, int radius) {
  // Determine color based on risk level
  cv::Scalar color;
  switch (cluster.GetRiskLevel()) {
    case RiskLevel::kRed:    color = cv::Scalar(0, 0, 255);     break; // Red
    case RiskLevel::kYellow: color = cv::Scalar(0, 255, 255);   break; // Yellow
    case RiskLevel::kGreen:  color = cv::Scalar(0, 255, 0);     break; // Green
    default:                 color = cv::Scalar(100, 100, 100); break; // Gray
  }

  // Scale distance to fit in display
  const float display_distance = std::min(
    cluster.GetDistance() * VisualizationConfig::kDistanceScale, 
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

}  // namespace brother_eye
