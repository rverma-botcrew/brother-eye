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

PointCloudProcessor::PointCloudProcessor() : next_id_(0), frame_count_(0) {
}

CloudT::Ptr PointCloudProcessor::ProcessPointCloud(const DDSPointCloud2& input_msg) {
  IncrementFrameCount();
  
  auto cloud_raw = ConvertToPcl(input_msg);
  auto cleaned = CleanCloud(cloud_raw);
  auto cluster_data = ExtractClustersWithBoundingBoxes(cleaned);
  
  // Extract centroids for tracking
  std::vector<cv::Point2f> centroids;
  for (const auto& data : cluster_data) {
    centroids.push_back(data.GetCentroid());
  }
  
  UpdateTracking(centroids);
  CleanupOldTrackers();
  
  return CreateTrackedPointCloud();
}

CloudT::Ptr PointCloudProcessor::ConvertToPcl(const DDSPointCloud2& msg) {
  const size_t point_count = msg.data().size() / msg.point_step();
  CloudT::Ptr cloud(new CloudT);
  cloud->width = static_cast<uint32_t>(point_count);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(point_count);

  const uint8_t* raw = msg.data().data();
  for (size_t i = 0; i < point_count; ++i) {
    const float* p = reinterpret_cast<const float*>(raw + i * msg.point_step());
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
    cloud->points[i].intensity = p[3];
  }
  return cloud;
}

CloudT::Ptr PointCloudProcessor::CleanCloud(const CloudT::Ptr& input_cloud) {
  // Voxel grid filtering
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.setLeafSize(kVoxelLeafSize, kVoxelLeafSize, kVoxelLeafSize);
  CloudT::Ptr voxel_filtered(new CloudT);
  voxel_grid.filter(*voxel_filtered);

  // Pass-through filtering
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud(voxel_filtered);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(kZFilterMin, kZFilterMax);
  CloudT::Ptr range_filtered(new CloudT);
  pass_through.filter(*range_filtered);

  // Ground plane removal using RANSAC
  pcl::SACSegmentation<PointT> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(kRansacMaxIterations);
  segmentation.setDistanceThreshold(kRansacDistanceThreshold);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  segmentation.setInputCloud(range_filtered);
  segmentation.segment(*inliers, *coefficients);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(range_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  CloudT::Ptr no_ground(new CloudT);
  extract.filter(*no_ground);

  // Statistical outlier removal
  pcl::StatisticalOutlierRemoval<PointT> statistical_outlier_removal;
  statistical_outlier_removal.setInputCloud(no_ground);
  statistical_outlier_removal.setMeanK(kStatisticalOutlierMeanK);
  statistical_outlier_removal.setStddevMulThresh(kStatisticalOutlierStddevMul);
  CloudT::Ptr cleaned(new CloudT);
  statistical_outlier_removal.filter(*cleaned);

  return cleaned;
}

std::vector<ClusterData> PointCloudProcessor::ExtractClustersWithBoundingBoxes(const CloudT::Ptr& cloud) {
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster;
  euclidean_cluster.setClusterTolerance(kClusterTolerance);
  euclidean_cluster.setMinClusterSize(kMinClusterSize);
  euclidean_cluster.setMaxClusterSize(kMaxClusterSize);
  euclidean_cluster.setSearchMethod(tree);
  euclidean_cluster.setInputCloud(cloud);
  euclidean_cluster.extract(cluster_indices);

  std::vector<ClusterData> clusters;

  for (const auto& indices : cluster_indices) {
    ClusterData cluster_data;
    
    // Calculate centroid and find min/max bounds
    float sum_x = 0, sum_y = 0, sum_z = 0;
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (int idx : indices.indices) {
      const PointT& p = cloud->points[idx];
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
      
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
      min_z = std::min(min_z, p.z);
      max_z = std::max(max_z, p.z);
      
      cluster_data.AddPointIndex(idx);
    }
    
    int num_points = indices.indices.size();
    cluster_data.SetCentroid(cv::Point2f(sum_x / num_points, sum_y / num_points));
    
    // Calculate 3D bounding box
    cluster_data.SetBboxCenter(cv::Point3f(
      (min_x + max_x) / 2.0f,
      (min_y + max_y) / 2.0f,
      (min_z + max_z) / 2.0f
    ));
    
    cluster_data.SetBboxSize(cv::Point3f(
      std::max(0.1f, max_x - min_x),  // Minimum 10cm width
      std::max(0.1f, max_y - min_y),  // Minimum 10cm depth
      std::max(0.1f, max_z - min_z)   // Minimum 10cm height
    ));
    
    clusters.push_back(cluster_data);
  }

  return clusters;
}

std::vector<ClusterInfo> PointCloudProcessor::AnalyzeClusterRisk(const std::vector<ClusterData>& cluster_data) {
  std::vector<ClusterInfo> cluster_info;
  
  for (size_t i = 0; i < cluster_data.size(); ++i) {
    ClusterInfo info(cluster_data[i].GetCentroid(), static_cast<int>(i));
    
    // Set bounding box information
    info.SetBboxCenter(cluster_data[i].GetBboxCenter());
    info.SetBboxSize(cluster_data[i].GetBboxSize());
    
    cluster_info.push_back(info);
    
    // Print cluster analysis
    std::cout << "[FILTER] 📊 Cluster " << info.GetId() 
              << " | Distance: " << std::fixed << std::setprecision(2) << info.GetDistance() << "m"
              << " | Angle: " << (info.GetAngle() * 180.0f / M_PI) << "°"
              << " | BBox: [" << info.GetBboxSize().x << "×" << info.GetBboxSize().y << "×" << info.GetBboxSize().z << "]m"
              << " | Risk: ";
    
    switch (info.GetRisk()) {
      case RiskLevel::kRed: std::cout << "🔴 RED"; break;
      case RiskLevel::kYellow: std::cout << "🟡 YELLOW"; break;
      case RiskLevel::kGreen: std::cout << "🟢 GREEN"; break;
      default: std::cout << "⚪ NONE"; break;
    }
    std::cout << std::endl;
  }
  
  return cluster_info;
}

void PointCloudProcessor::UpdateTracking(const std::vector<cv::Point2f>& centroids) {
  // Predict all tracker positions before matching
  for (auto& [id, obj] : tracked_objects_) {
    obj.Predict();
  }

  // Simple nearest match tracking
  std::set<int> matched_ids;
  for (const auto& centroid : centroids) {
    int best_id = -1;
    float best_distance = std::max(0.4f, 1.0f - 0.05f * tracked_objects_.size());
    
    // Find existing object closest to centroid
    for (auto& [id, obj] : tracked_objects_) {
      float distance = obj.DistanceTo(centroid);
      if (distance < best_distance && matched_ids.count(id) == 0) {
        best_distance = distance;
        best_id = id;
      }
    }

    if (best_id == -1) {
      // New object
      tracked_objects_[next_id_] = TrackedObject(centroid);
      matched_ids.insert(next_id_);
      ++next_id_;
    } else {
      // Existing object update
      tracked_objects_[best_id].Update(centroid);
      matched_ids.insert(best_id);
    }
  }
}

void PointCloudProcessor::CleanupOldTrackers() {
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if (it->second.GetLostFrames() > kMaxLostFrames) {
      it = tracked_objects_.erase(it);
    } else {
      ++it;
    }
  }
}

CloudT::Ptr PointCloudProcessor::CreateTrackedPointCloud() {
  CloudT::Ptr clustered(new CloudT);
  for (const auto& [id, obj] : tracked_objects_) {
    PointT point;
    point.x = obj.GetLastCentroid().x;
    point.y = obj.GetLastCentroid().y;
    point.z = 0.0f;
    point.intensity = 200.0f + id % 55;  // Unique intensity per ID
    clustered->points.push_back(point);
  }
  clustered->width = clustered->points.size();
  clustered->height = 1;
  clustered->is_dense = true;
  
  return clustered;
}

void PointCloudProcessor::DisplayClusterRiskUi(const std::vector<ClusterInfo>& clusters) {
  const int radius = 200;
  const cv::Point center(radius + 10, radius + 10);
  const int img_size = 2 * radius + 20;

  cv::Mat img(img_size, img_size, CV_8UC3, cv::Scalar(30, 30, 30));

  // Draw distance circles
  for (int r = 50; r <= radius; r += 50) {
    cv::circle(img, center, r, cv::Scalar(80, 80, 80), 1);
  }
  
  // Draw angle lines every 30 degrees
  for (int angle = 0; angle < 360; angle += 30) {
    float rad = angle * M_PI / 180.0f;
    cv::Point end(center.x + radius * std::cos(rad),
                  center.y + radius * std::sin(rad));
    cv::line(img, center, end, cv::Scalar(80, 80, 80), 1);
  }

  // Draw clusters
  for (const auto& cluster : clusters) {
    cv::Scalar color;
    switch (cluster.GetRisk()) {
      case RiskLevel::kRed: color = cv::Scalar(0, 0, 255); break;
      case RiskLevel::kYellow: color = cv::Scalar(0, 255, 255); break;
      case RiskLevel::kGreen: color = cv::Scalar(0, 255, 0); break;
      default: color = cv::Scalar(100, 100, 100); break;
    }

    // Scale distance to fit in display (assuming max 10m range)
    float display_distance = std::min(cluster.GetDistance() * 20.0f, static_cast<float>(radius));
    cv::Point cluster_pos(
      center.x + display_distance * std::cos(cluster.GetAngle()),
      center.y + display_distance * std::sin(cluster.GetAngle())
    );

    cv::circle(img, cluster_pos, 8, color, -1);
    cv::circle(img, cluster_pos, 8, cv::Scalar(255, 255, 255), 2);
    
    // Add cluster ID text
    cv::putText(img, std::to_string(cluster.GetId()), 
                cv::Point(cluster_pos.x + 10, cluster_pos.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }

  // Draw center point
  cv::circle(img, center, 3, cv::Scalar(255, 255, 255), -1);
  
  // Add legend
  cv::putText(img, "Cluster Risk Analysis", cv::Point(10, 20), 
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

  cv::imshow("Cluster Risk Analysis", img);
  cv::waitKey(1);
}

}  // namespace brother_eye
