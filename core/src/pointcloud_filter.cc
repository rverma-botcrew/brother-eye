// --- Includes ---
#include <ddscxx/dds/dds.hpp>
#include "dds_pcl.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <iostream>
#include <csignal>
#include <thread>
#include <limits>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <exception>
#include <iomanip>
#include <chrono>
#include <fstream>

// --- Aliases ---
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;
using DDSClusterArray = pcl_dds_msgs::ClusterArray;
using DDSClusterInfo = pcl_dds_msgs::ClusterInfo;
using DDSPoint2D = pcl_dds_msgs::Point2D;
using DDSPoint3D = pcl_dds_msgs::Point3D;
using DDSBoundingBox3D = pcl_dds_msgs::BoundingBox3D;
using DDSBoundingBoxArray = pcl_dds_msgs::BoundingBoxArray;
using DDSVector3 = pcl_dds_msgs::Vector3;
using DDSPose = pcl_dds_msgs::Pose;
using DDSQuaternion = pcl_dds_msgs::Quaternion;

// --- Constants ---
constexpr int RISK_HOLD_FRAMES = 5;

// --- Risk Level Enum ---
enum class RiskLevel { NONE, GREEN, YELLOW, RED };

// --- Cluster Data Structure ---
struct ClusterInfo {
  cv::Point2f centroid;
  float distance;
  float angle;  // in radians
  RiskLevel risk;
  int id;
  
  // 3D Bounding Box data
  cv::Point3f bbox_center;
  cv::Point3f bbox_size;  // width, height, depth
  
  ClusterInfo(const cv::Point2f& c, int cluster_id) 
    : centroid(c), id(cluster_id) {
    distance = std::hypot(c.x, c.y);
    angle = std::atan2(c.y, c.x);
    
    // Calculate risk based on distance
    if (distance < 1.5f) {
      risk = RiskLevel::RED;
    } else if (distance < 2.0f) {
      risk = RiskLevel::YELLOW;
    } else if (distance < 5.0f) {
      risk = RiskLevel::GREEN;
    } else {
      risk = RiskLevel::NONE;
    }
    
    // Default bounding box (will be calculated later)
    bbox_center = cv::Point3f(c.x, c.y, 0.5f); // Assume 0.5m height center
    bbox_size = cv::Point3f(0.5f, 0.5f, 1.0f);  // Default size: 0.5x0.5x1.0m
  }
};

class KalmanTracker {
  cv::KalmanFilter kf;
  cv::Mat state;      // [x, y, vx, vy]
  cv::Mat meas;       // [x, y]

public:
  KalmanTracker() {
    kf = cv::KalmanFilter(4, 2);
    state = cv::Mat::zeros(4, 1, CV_32F);
    meas = cv::Mat::zeros(2, 1, CV_32F);

    kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
                            1, 0, 1, 0,
                            0, 1, 0, 1,
                            0, 0, 1, 0,
                            0, 0, 0, 1);
    kf.measurementMatrix = cv::Mat::eye(2, 4, CV_32F);
    setIdentity(kf.processNoiseCov, cv::Scalar(1e-1));    // smoother prediction
    setIdentity(kf.measurementNoiseCov, cv::Scalar(5e-2)); // trust measurements less
    setIdentity(kf.errorCovPost, cv::Scalar(1));
  }

  cv::Point2f update(const cv::Point2f& pt) {
    meas.at<float>(0) = pt.x;
    meas.at<float>(1) = pt.y;

    kf.correct(meas);
    cv::Mat prediction = kf.predict();

    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
  }

  // Provide direct prediction without measurement update
  cv::Point2f predict() {
    cv::Mat prediction = kf.predict();
    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
  }
};


// --- Kalman Tracker Class ---
class TrackedObject {
public:
  KalmanTracker kf;
  cv::Point2f last_centroid;
  int age = 0;
  int lost_frames = 0;
  bool active = true;

  TrackedObject() = default;

  TrackedObject(const cv::Point2f& init)
    : kf(), last_centroid(init), age(0), lost_frames(0), active(true) {
  }

  void update(const cv::Point2f& new_pos) {
    last_centroid = kf.update(new_pos);
    lost_frames = 0;
    ++age;
  }

  void predict() {
    last_centroid = kf.predict();
    ++lost_frames;
  }

  float distanceTo(const cv::Point2f& p) const {
    return cv::norm(last_centroid - p);
  }
};


// --- Globals ---
std::map<int, TrackedObject> tracked_objects;
int next_id = 0;

// std::vector<cv::Point2f> tracked_centroids;
static bool running = true;
void sigint_handler(int) { running = false; }

// --- DDS → PCL Conversion ---
static CloudT::Ptr convertToPCL(const DDSPointCloud2 &msg) {
  const size_t point_count = msg.data().size() / msg.point_step();
  CloudT::Ptr cloud(new CloudT);
  cloud->width = static_cast<uint32_t>(point_count);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(point_count);

  const uint8_t *raw = msg.data().data();
  for (size_t i = 0; i < point_count; ++i) {
    const float *p = reinterpret_cast<const float *>(raw + i * msg.point_step());
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
    cloud->points[i].intensity = p[3];
  }
  return cloud;
}

// --- Clean PointCloud ---
static CloudT::Ptr cleanCloud(const CloudT::Ptr &in) {
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(0.05f, 0.05f, 0.05f);
  CloudT::Ptr vg_cloud(new CloudT);
  vg.filter(*vg_cloud);

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(vg_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-2.0f, 10.0f);
  CloudT::Ptr range_cloud(new CloudT);
  pass.filter(*range_cloud);

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setInputCloud(range_cloud);
  seg.segment(*inliers, *coeff);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(range_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  CloudT::Ptr no_ground(new CloudT);
  extract.filter(*no_ground);

  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(no_ground);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0f);
  CloudT::Ptr cleaned(new CloudT);
  sor.filter(*cleaned);

  return cleaned;
}


std::vector<cv::Point2f> extractCentroids(const CloudT::Ptr& cloud) {
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.2);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(5000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::vector<cv::Point2f> centroids;

  for (const auto& indices : cluster_indices) {
    float sum_x = 0, sum_y = 0;
    for (int idx : indices.indices) {
      const PointT& p = cloud->points[idx];
      sum_x += p.x;
      sum_y += p.y;
    }
    int N = indices.indices.size();
    centroids.emplace_back(sum_x / N, sum_y / N);
  }

  return centroids;
}

// --- Extract Clusters with Bounding Box Information ---
struct ClusterData {
  cv::Point2f centroid;
  cv::Point3f bbox_center;
  cv::Point3f bbox_size;
  std::vector<int> point_indices;
};

std::vector<ClusterData> extractClustersWithBoundingBoxes(const CloudT::Ptr& cloud) {
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.2);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(5000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

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
      
      cluster_data.point_indices.push_back(idx);
    }
    
    int N = indices.indices.size();
    cluster_data.centroid = cv::Point2f(sum_x / N, sum_y / N);
    
    // Calculate 3D bounding box
    cluster_data.bbox_center = cv::Point3f(
      (min_x + max_x) / 2.0f,
      (min_y + max_y) / 2.0f,
      (min_z + max_z) / 2.0f
    );
    
    cluster_data.bbox_size = cv::Point3f(
      std::max(0.1f, max_x - min_x), // Minimum 10cm width
      std::max(0.1f, max_y - min_y), // Minimum 10cm depth
      std::max(0.1f, max_z - min_z)  // Minimum 10cm height
    );
    
    clusters.push_back(cluster_data);
  }

  return clusters;
}


// --- Cluster Analysis ---
std::vector<ClusterInfo> analyzeClusterRisk(const std::vector<ClusterData>& cluster_data) {
  std::vector<ClusterInfo> cluster_info;
  
  for (size_t i = 0; i < cluster_data.size(); ++i) {
    ClusterInfo info(cluster_data[i].centroid, static_cast<int>(i));
    
    // Set bounding box information
    info.bbox_center = cluster_data[i].bbox_center;
    info.bbox_size = cluster_data[i].bbox_size;
    
    cluster_info.push_back(info);
    
    // Print cluster analysis
    std::cout << "[FILTER] 📊 Cluster " << info.id 
              << " | Distance: " << std::fixed << std::setprecision(2) << info.distance << "m"
              << " | Angle: " << (info.angle * 180.0f / M_PI) << "°"
              << " | BBox: [" << info.bbox_size.x << "×" << info.bbox_size.y << "×" << info.bbox_size.z << "]m"
              << " | Risk: ";
    
    switch (info.risk) {
      case RiskLevel::RED: std::cout << "🔴 RED"; break;
      case RiskLevel::YELLOW: std::cout << "🟡 YELLOW"; break;
      case RiskLevel::GREEN: std::cout << "🟢 GREEN"; break;
      default: std::cout << "⚪ NONE"; break;
    }
    std::cout << std::endl;
  }
  
  return cluster_info;
}

// --- Validation functions ---
bool isValidFloat(float value) {
  return std::isfinite(value) && !std::isnan(value);
}

bool validatePoint2D(const cv::Point2f& point) {
  return isValidFloat(point.x) && isValidFloat(point.y);
}

bool validatePoint3D(const cv::Point3f& point) {
  return isValidFloat(point.x) && isValidFloat(point.y) && isValidFloat(point.z);
}

// --- JSON Serialization Functions ---
std::string escapeJsonString(const std::string& input) {
  std::ostringstream ss;
  for (char c : input) {
    switch (c) {
      case '"': ss << "\\\""; break;
      case '\\': ss << "\\\\"; break;
      case '\b': ss << "\\b"; break;
      case '\f': ss << "\\f"; break;
      case '\n': ss << "\\n"; break;
      case '\r': ss << "\\r"; break;
      case '\t': ss << "\\t"; break;
      default: ss << c; break;
    }
  }
  return ss.str();
}

std::string formatFloat(float value, int precision = 3) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  return ss.str();
}

std::string clusterInfoToJson(const ClusterInfo& cluster, int tracked_id = -1, int age = 0, int lost_frames = 0) {
  std::ostringstream json;
  json << "{\n";
  json << "    \"id\": " << cluster.id << ",\n";
  json << "    \"tracked_id\": " << tracked_id << ",\n";
  json << "    \"centroid\": {\n";
  json << "      \"x\": " << formatFloat(cluster.centroid.x) << ",\n";
  json << "      \"y\": " << formatFloat(cluster.centroid.y) << "\n";
  json << "    },\n";
  json << "    \"distance\": " << formatFloat(cluster.distance) << ",\n";
  json << "    \"angle_radians\": " << formatFloat(cluster.angle) << ",\n";
  json << "    \"angle_degrees\": " << formatFloat(cluster.angle * 180.0f / M_PI) << ",\n";
  
  // Risk level as both enum and string
  std::string risk_str;
  int risk_level = 0;
  switch (cluster.risk) {
    case RiskLevel::RED: risk_str = "RED"; risk_level = 3; break;
    case RiskLevel::YELLOW: risk_str = "YELLOW"; risk_level = 2; break;
    case RiskLevel::GREEN: risk_str = "GREEN"; risk_level = 1; break;
    default: risk_str = "NONE"; risk_level = 0; break;
  }
  
  json << "    \"risk\": {\n";
  json << "      \"level\": " << risk_level << ",\n";
  json << "      \"name\": \"" << risk_str << "\"\n";
  json << "    },\n";
  
  // Bounding box
  json << "    \"bounding_box\": {\n";
  json << "      \"center\": {\n";
  json << "        \"x\": " << formatFloat(cluster.bbox_center.x) << ",\n";
  json << "        \"y\": " << formatFloat(cluster.bbox_center.y) << ",\n";
  json << "        \"z\": " << formatFloat(cluster.bbox_center.z) << "\n";
  json << "      },\n";
  json << "      \"size\": {\n";
  json << "        \"width\": " << formatFloat(cluster.bbox_size.x) << ",\n";
  json << "        \"height\": " << formatFloat(cluster.bbox_size.y) << ",\n";
  json << "        \"depth\": " << formatFloat(cluster.bbox_size.z) << "\n";
  json << "      },\n";
  json << "      \"volume\": " << formatFloat(cluster.bbox_size.x * cluster.bbox_size.y * cluster.bbox_size.z) << "\n";
  json << "    },\n";
  
  // Tracking information
  json << "    \"tracking\": {\n";
  json << "      \"age\": " << age << ",\n";
  json << "      \"lost_frames\": " << lost_frames << "\n";
  json << "    }\n";
  json << "  }";
  
  return json.str();
}

std::string clustersToJson(const std::vector<ClusterInfo>& clusters, 
                          const std::map<int, TrackedObject>& tracked_objects,
                          size_t frame_number) {
  std::ostringstream json;
  
  // Get current timestamp
  auto now = std::chrono::high_resolution_clock::now();
  auto time_since_epoch = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
  
  json << "{\n";
  json << "  \"timestamp\": {\n";
  json << "    \"seconds\": " << seconds.count() << ",\n";
  json << "    \"milliseconds\": " << milliseconds.count() << ",\n";
  json << "    \"iso8601\": \"" << std::to_string(seconds.count()) << "." << std::setfill('0') << std::setw(3) << milliseconds.count() << "\"\n";
  json << "  },\n";
  json << "  \"frame_number\": " << frame_number << ",\n";
  json << "  \"frame_id\": \"base_link\",\n";
  json << "  \"cluster_count\": " << clusters.size() << ",\n";
  json << "  \"tracked_object_count\": " << tracked_objects.size() << ",\n";
  
  // Statistics
  size_t red_count = 0, yellow_count = 0, green_count = 0, none_count = 0;
  float min_distance = std::numeric_limits<float>::max();
  float max_distance = 0.0f;
  
  for (const auto& cluster : clusters) {
    switch (cluster.risk) {
      case RiskLevel::RED: red_count++; break;
      case RiskLevel::YELLOW: yellow_count++; break;
      case RiskLevel::GREEN: green_count++; break;
      default: none_count++; break;
    }
    min_distance = std::min(min_distance, cluster.distance);
    max_distance = std::max(max_distance, cluster.distance);
  }
  
  if (clusters.empty()) {
    min_distance = 0.0f;
  }
  
  json << "  \"statistics\": {\n";
  json << "    \"risk_distribution\": {\n";
  json << "      \"red\": " << red_count << ",\n";
  json << "      \"yellow\": " << yellow_count << ",\n";
  json << "      \"green\": " << green_count << ",\n";
  json << "      \"none\": " << none_count << "\n";
  json << "    },\n";
  json << "    \"distance_range\": {\n";
  json << "      \"min\": " << formatFloat(min_distance) << ",\n";
  json << "      \"max\": " << formatFloat(max_distance) << "\n";
  json << "    }\n";
  json << "  },\n";
  
  json << "  \"clusters\": [\n";
  
  // Add cluster data
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& cluster = clusters[i];
    
    // Find matching tracked object
    int tracked_id = -1;
    int age = 0;
    int lost_frames = 0;
    
    for (const auto& [id, obj] : tracked_objects) {
      if (cv::norm(cluster.centroid - obj.last_centroid) < 0.3f) {
        tracked_id = id;
        age = obj.age;
        lost_frames = obj.lost_frames;
        break;
      }
    }
    
    json << clusterInfoToJson(cluster, tracked_id, age, lost_frames);
    if (i < clusters.size() - 1) {
      json << ",";
    }
    json << "\n";
  }
  
  json << "  ]\n";
  json << "}";
  
  return json.str();
}

// --- JSON Publishing Functions ---
void publishJsonToFile(const std::string& json_data, const std::string& filename = "/tmp/clusters.json") {
  try {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << json_data;
      file.close();
      std::cout << "[FILTER] 📄 Published JSON to " << filename << "\n";
    } else {
      std::cerr << "[FILTER] ❌ Failed to open JSON file: " << filename << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "[FILTER] ❌ JSON file write error: " << e.what() << "\n";
  }
}

void publishJsonToConsole(const std::string& json_data) {
  std::cout << "[FILTER] 📄 JSON Output:\n" << json_data << "\n" << std::endl;
}

// --- Convert to DDS Message ---
DDSClusterArray convertToClusterArray(const std::vector<ClusterInfo>& clusters, 
                                     const std::map<int, TrackedObject>& tracked_objects) {
  DDSClusterArray msg;
  
  // Set header with current time
  auto now = std::chrono::high_resolution_clock::now();
  auto time_since_epoch = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch - seconds);
  
  msg.header().stamp().sec(static_cast<int32_t>(seconds.count()));
  msg.header().stamp().nanosec(static_cast<uint32_t>(nanoseconds.count()));
  msg.header().frame_id("base_link");
  
  // Limit cluster count to avoid DDS sequence issues
  const size_t max_clusters = 512; // Well under the 1024 limit
  size_t cluster_count = std::min(tracked_objects.size(), max_clusters);
  
  std::cout << "[FILTER] 🔍 Converting " << tracked_objects.size() << " tracked objects to DDS (max " << max_clusters << ")\n";
  
  // Convert clusters to DDS format - first count valid ones
  std::vector<std::pair<int, TrackedObject>> valid_objects;
  for (const auto& [id, obj] : tracked_objects) {
    if (valid_objects.size() >= max_clusters) {
      std::cout << "[FILTER] ⚠️ Reached maximum cluster limit, stopping at " << max_clusters << "\n";
      break;
    }
    
    // Validate centroid data
    if (!validatePoint2D(obj.last_centroid)) {
      std::cerr << "[FILTER] ⚠️ Invalid centroid for cluster " << id 
                << " [" << obj.last_centroid.x << ", " << obj.last_centroid.y << "], skipping\n";
      continue;
    }
    
    // Calculate distance and angle
    float distance = std::hypot(obj.last_centroid.x, obj.last_centroid.y);
    float angle = std::atan2(obj.last_centroid.y, obj.last_centroid.x);
    
    // Validate calculated values
    if (!isValidFloat(distance) || !isValidFloat(angle)) {
      std::cerr << "[FILTER] ⚠️ Invalid distance/angle for cluster " << id 
                << " [dist=" << distance << ", angle=" << angle << "], skipping\n";
      continue;
    }
    
    // Validate object age and lost_frames (should be non-negative)
    if (obj.age < 0 || obj.lost_frames < 0) {
      std::cerr << "[FILTER] ⚠️ Invalid age/lost_frames for cluster " << id 
                << " [age=" << obj.age << ", lost=" << obj.lost_frames << "], skipping\n";
      continue;
    }
    
    valid_objects.push_back({id, obj});
  }
  
  std::cout << "[FILTER] 🔍 Found " << valid_objects.size() << " valid clusters for DDS\n";
  
  if (valid_objects.empty()) {
    std::cout << "[FILTER] ⚠️ No valid clusters to publish\n";
    msg.clusters().resize(0);
    return msg;
  }
  
  msg.clusters().resize(valid_objects.size());
  size_t idx = 0;
  
  for (const auto& [id, obj] : valid_objects) {
    std::cout << "[FILTER] 🔍 Processing cluster " << id << " at index " << idx << "\n";
    
    DDSClusterInfo& dds_cluster = msg.clusters()[idx];
    
    // Set centroid with additional validation
    float centroid_x = obj.last_centroid.x;
    float centroid_y = obj.last_centroid.y;
    
    // Clamp extreme values
    centroid_x = std::max(-100.0f, std::min(100.0f, centroid_x));
    centroid_y = std::max(-100.0f, std::min(100.0f, centroid_y));
    
    dds_cluster.centroid().x(centroid_x);
    dds_cluster.centroid().y(centroid_y);
    
    // Calculate distance and angle with clamping
    float distance = std::hypot(centroid_x, centroid_y);
    float angle = std::atan2(centroid_y, centroid_x);
    
    // Clamp distance to reasonable range
    distance = std::max(0.0f, std::min(200.0f, distance));
    
    dds_cluster.distance(distance);
    dds_cluster.angle(angle);
    dds_cluster.id(id);
    dds_cluster.age(std::max(0, obj.age));
    dds_cluster.lost_frames(std::max(0, obj.lost_frames));
    
    // Set risk level
    uint8_t risk_level = 0; // NONE
    if (distance < 1.5f) {
      risk_level = 3; // RED
    } else if (distance < 2.0f) {
      risk_level = 2; // YELLOW
    } else if (distance < 5.0f) {
      risk_level = 1; // GREEN
    }
    dds_cluster.risk_level(risk_level);
    
    // Set bounding box (find corresponding cluster info or use defaults)
    cv::Point3f bbox_center(centroid_x, centroid_y, 0.5f);
    cv::Point3f bbox_size(0.5f, 0.5f, 1.0f);
    
    // Try to find matching cluster info for bounding box data
    for (const auto& cluster : clusters) {
      if (cv::norm(cluster.centroid - obj.last_centroid) < 0.3f) {
        if (validatePoint3D(cluster.bbox_center) && validatePoint3D(cluster.bbox_size)) {
          bbox_center = cluster.bbox_center;
          bbox_size = cluster.bbox_size;
        }
        break;
      }
    }
    
    // Clamp bounding box center to reasonable values
    bbox_center.x = std::max(-100.0f, std::min(100.0f, bbox_center.x));
    bbox_center.y = std::max(-100.0f, std::min(100.0f, bbox_center.y));
    bbox_center.z = std::max(-10.0f, std::min(10.0f, bbox_center.z));
    
    // Ensure reasonable bounding box size (minimum 1cm, maximum 50m)
    bbox_size.x = std::max(0.01f, std::min(50.0f, bbox_size.x));
    bbox_size.y = std::max(0.01f, std::min(50.0f, bbox_size.y));
    bbox_size.z = std::max(0.01f, std::min(50.0f, bbox_size.z));
    
    // Set bounding box center (position)
    dds_cluster.bounding_box().center().position().x(bbox_center.x);
    dds_cluster.bounding_box().center().position().y(bbox_center.y);
    dds_cluster.bounding_box().center().position().z(bbox_center.z);
    
    // Set bounding box orientation (identity quaternion for axis-aligned boxes)
    dds_cluster.bounding_box().center().orientation().x(0.0f);
    dds_cluster.bounding_box().center().orientation().y(0.0f);
    dds_cluster.bounding_box().center().orientation().z(0.0f);
    dds_cluster.bounding_box().center().orientation().w(1.0f);
    
    // Set bounding box size
    dds_cluster.bounding_box().size().x(bbox_size.x);
    dds_cluster.bounding_box().size().y(bbox_size.y);
    dds_cluster.bounding_box().size().z(bbox_size.z);
    
    std::cout << "[FILTER] 🔍 Cluster " << id << " data: centroid=[" << centroid_x << ", " << centroid_y 
              << "], dist=" << distance << ", bbox_size=[" << bbox_size.x << ", " << bbox_size.y << ", " << bbox_size.z << "]\n";
    
    ++idx;
  }
  
  std::cout << "[FILTER] ✅ Converted " << idx << " clusters to DDS message\n";
  return msg;
}

// --- Convert to Bounding Box Array ---
DDSBoundingBoxArray convertToBoundingBoxArray(const std::vector<ClusterInfo>& clusters) {
  DDSBoundingBoxArray msg;
  
  // Set header with current time
  auto now = std::chrono::high_resolution_clock::now();
  auto time_since_epoch = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch - seconds);
  
  msg.header().stamp().sec(static_cast<int32_t>(seconds.count()));
  msg.header().stamp().nanosec(static_cast<uint32_t>(nanoseconds.count()));
  msg.header().frame_id("base_link");
  
  // Convert bounding boxes - first filter valid ones
  std::vector<ClusterInfo> valid_clusters;
  for (const auto& cluster : clusters) {
    if (validatePoint2D(cluster.centroid)) {
      valid_clusters.push_back(cluster);
    } else {
      std::cerr << "[FILTER] ⚠️ Invalid cluster centroid, skipping from bbox array\n";
    }
  }
  
  msg.boxes().resize(valid_clusters.size());
  
  for (size_t i = 0; i < valid_clusters.size(); ++i) {
    const auto& cluster = valid_clusters[i];
    DDSBoundingBox3D& bbox = msg.boxes()[i];
    
    // Validate cluster data
    if (!validatePoint3D(cluster.bbox_center) || !validatePoint3D(cluster.bbox_size)) {
      std::cerr << "[FILTER] ⚠️ Invalid bounding box for cluster " << i << ", using defaults\n";
      // Set default center position
      bbox.center().position().x(cluster.centroid.x);
      bbox.center().position().y(cluster.centroid.y);
      bbox.center().position().z(0.5f);
      
      // Set default size
      bbox.size().x(0.5f);
      bbox.size().y(0.5f);
      bbox.size().z(1.0f);
    } else {
      // Set center position
      bbox.center().position().x(cluster.bbox_center.x);
      bbox.center().position().y(cluster.bbox_center.y);
      bbox.center().position().z(cluster.bbox_center.z);
      
      // Set size with minimum bounds
      bbox.size().x(std::max(0.1f, cluster.bbox_size.x));
      bbox.size().y(std::max(0.1f, cluster.bbox_size.y));
      bbox.size().z(std::max(0.1f, cluster.bbox_size.z));
    }
    
    // Set orientation (identity quaternion for axis-aligned boxes)
    bbox.center().orientation().x(0.0f);
    bbox.center().orientation().y(0.0f);
    bbox.center().orientation().z(0.0f);
    bbox.center().orientation().w(1.0f);
  }
  
  return msg;
}

// --- Publish Cloud to DDS ---
static void publishCloud(const CloudT::Ptr &cloud,
                         dds::pub::DataWriter<DDSPointCloud2> &writer,
                         const DDSPointCloud2 &proto) {
  // 💣 Bail out if empty cloud
  if (cloud->points.empty()) {
    std::cerr << "[FILTER] ⚠️ Skipping publish: empty cloud\n";
    return;
  }

  // 💣 Bail out if proto is invalid (this often causes DDS crash)
  if (proto.point_step() == 0) {
    std::cerr << "[FILTER] ❌ Invalid proto: point_step=0\n";
    return;
  }

  DDSPointCloud2 out = proto;
  out.width(cloud->points.size());
  out.height(1);
  out.row_step(out.point_step() * out.width());
  out.is_dense(false);
  out.data().resize(out.row_step());

  uint8_t *dst = out.data().data();
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float *p_dst = reinterpret_cast<float *>(dst + i * out.point_step());
    const auto &p = cloud->points[i];
    p_dst[0] = p.x;
    p_dst[1] = p.y;
    p_dst[2] = p.z;
    p_dst[3] = p.intensity;
  }

  try {
    writer.write(out);
  } catch (const dds::core::Exception &e) {
    std::cerr << "[FILTER] ❌ DDS Write Exception: " << e.what() << "\n";
  } catch (const std::exception &e) {
    std::cerr << "[FILTER] ❌ std::exception in DDS write: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "[FILTER] ❌ Unknown exception in DDS write\n";
  }
}


// --- Visual UI for Cluster Risk ---
void displayClusterRiskUI(const std::vector<ClusterInfo>& clusters) {
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
    switch (cluster.risk) {
      case RiskLevel::RED: color = cv::Scalar(0, 0, 255); break;
      case RiskLevel::YELLOW: color = cv::Scalar(0, 255, 255); break;
      case RiskLevel::GREEN: color = cv::Scalar(0, 255, 0); break;
      default: color = cv::Scalar(100, 100, 100); break;
    }

    // Scale distance to fit in display (assuming max 10m range)
    float display_distance = std::min(cluster.distance * 20.0f, static_cast<float>(radius));
    cv::Point cluster_pos(
      center.x + display_distance * std::cos(cluster.angle),
      center.y + display_distance * std::sin(cluster.angle)
    );

    cv::circle(img, cluster_pos, 8, color, -1);
    cv::circle(img, cluster_pos, 8, cv::Scalar(255, 255, 255), 2);
    
    // Add cluster ID text
    cv::putText(img, std::to_string(cluster.id), 
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

// --- Main ---
int main() {
  std::signal(SIGINT, sigint_handler);
  std::cout << "[FILTER] 🚀 Initializing PointCloud filter node...\n";

  dds::domain::DomainParticipant dp(0);
  dds::topic::Topic<DDSPointCloud2> in_topic(dp, "dds_raw_points");
  dds::topic::Topic<DDSPointCloud2> out_topic(dp, "dds_clustered_points");
  dds::topic::Topic<DDSClusterArray> cluster_topic(dp, "dds_cluster_analysis");
  dds::topic::Topic<DDSBoundingBoxArray> bbox_topic(dp, "dds_bounding_boxes");
  dds::sub::Subscriber sub(dp);
  dds::pub::Publisher pub(dp);

  auto reader_qos = dds::sub::qos::DataReaderQos()
    << dds::core::policy::Reliability::Reliable()
    << dds::core::policy::History::KeepLast(10);

  auto writer_qos = dds::pub::qos::DataWriterQos()
    << dds::core::policy::Reliability::Reliable()
    << dds::core::policy::History::KeepLast(10);

  dds::sub::DataReader<DDSPointCloud2> reader(sub, in_topic, reader_qos);
  dds::pub::DataWriter<DDSPointCloud2> writer(pub, out_topic, writer_qos);
  dds::pub::DataWriter<DDSClusterArray> cluster_writer(pub, cluster_topic, writer_qos);
  dds::pub::DataWriter<DDSBoundingBoxArray> bbox_writer(pub, bbox_topic, writer_qos);

  std::cout << "[FILTER] 🔗 Subscribed to DDS topic: dds_raw_points\n";
  std::cout << "[FILTER] 📨 Publishing to DDS topic: dds_clustered_points\n";
  std::cout << "[FILTER] 📊 Publishing cluster analysis to DDS topic: dds_cluster_analysis\n";
  std::cout << "[FILTER] 📦 Publishing bounding boxes to DDS topic: dds_bounding_boxes\n";

  size_t frame_count = 0;
  while (running) {
    auto samples = reader.take();
    if (samples.length() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    for (auto &s : samples) {
      const auto &msg = s.data();
      std::cout << "[FILTER] 📦 Frame #" << ++frame_count
                << " | Points = " << msg.data().size() / msg.point_step() << std::endl;

      auto cloud_raw = convertToPCL(msg);
      auto cleaned = cleanCloud(cloud_raw);
      auto cluster_data = extractClustersWithBoundingBoxes(cleaned);
      
      // Extract just centroids for tracking
      std::vector<cv::Point2f> centroids;
      for (const auto& data : cluster_data) {
        centroids.push_back(data.centroid);
      }

      // Predict all tracker positions before matching
      for (auto& [id, obj] : tracked_objects) {
        obj.predict();
      }

      // Simple nearest match tracking
      std::set<int> matched_ids;
      for (const auto& c : centroids) {
        int best_id = -1;
        float best_dist = std::max(0.4f, 1.0f - 0.05f * tracked_objects.size());
        // Find existing object closest to centroid
        for (auto& [id, obj] : tracked_objects) {
          float d = obj.distanceTo(c);
          if (d < best_dist && matched_ids.count(id) == 0) {
            best_dist = d;
            best_id = id;
          }
        }

        if (best_id == -1) {
          // New object
          tracked_objects[next_id] = TrackedObject(c);
          matched_ids.insert(next_id);
          ++next_id;
        } else {
          // Existing object update
          tracked_objects[best_id].update(c);
          matched_ids.insert(best_id);
        }
      }

      // Optionally: clean up old trackers not updated
      for (auto it = tracked_objects.begin(); it != tracked_objects.end();) {
        if (it->second.lost_frames > 5) {
          it = tracked_objects.erase(it);
        } else {
          ++it;
        }
      }

      // auto clustered = extractClusters(cleaned);
      CloudT::Ptr clustered(new CloudT);
      for (const auto& [id, obj] : tracked_objects) {
        PointT p;
        p.x = obj.last_centroid.x;
        p.y = obj.last_centroid.y;
        p.z = 0.0f;
        p.intensity = 200.0f + id % 55;  // Uniqueish intensity per ID
        clustered->points.push_back(p);
      }
      clustered->width = clustered->points.size();
      clustered->height = 1;
      clustered->is_dense = true;

      // Analyze cluster risk with distance and angle
      auto cluster_analysis = analyzeClusterRisk(cluster_data);
      displayClusterRiskUI(cluster_analysis);

      // Export cluster analysis as JSON
      std::string json_data = clustersToJson(cluster_analysis, tracked_objects, frame_count);
      publishJsonToFile(json_data, "/tmp/clusters.json");

      // Publish cluster analysis over DDS
      auto cluster_msg = convertToClusterArray(cluster_analysis, tracked_objects);
      try {
        cluster_writer.write(cluster_msg);
        std::cout << "[FILTER] 📊 Published cluster analysis (" << cluster_msg.clusters().size() << " clusters)\n";
      } catch (const dds::core::Exception &e) {
        std::cerr << "[FILTER] ❌ DDS cluster write exception: " << e.what() << "\n";
      } catch (const std::exception &e) {
        std::cerr << "[FILTER] ❌ std::exception in cluster write: " << e.what() << "\n";
      } catch (...) {
        std::cerr << "[FILTER] ❌ Unknown exception in cluster write\n";
      }

      // Publish bounding boxes over DDS
      auto bbox_msg = convertToBoundingBoxArray(cluster_analysis);
      try {
        bbox_writer.write(bbox_msg);
        std::cout << "[FILTER] 📦 Published bounding boxes (" << bbox_msg.boxes().size() << " boxes)\n";
      } catch (const dds::core::Exception &e) {
        std::cerr << "[FILTER] ❌ DDS bbox write exception: " << e.what() << "\n";
      } catch (const std::exception &e) {
        std::cerr << "[FILTER] ❌ std::exception in bbox write: " << e.what() << "\n";
      } catch (...) {
        std::cerr << "[FILTER] ❌ Unknown exception in bbox write\n";
      }

      try {
        publishCloud(clustered, writer, msg);
        std::cout << "[FILTER] ✅ Published clustered cloud\n";
      } catch (const dds::core::Exception &e) {
        std::cerr << "[FILTER] ❌ DDS exception: " << e.what() << "\n";
      } catch (const std::exception &e) {
        std::cerr << "[FILTER] ❌ std::exception: " << e.what() << "\n";
      } catch (...) {
        std::cerr << "[FILTER] ❌ unknown exception\n";
      }
    }
  }

  std::cout << "[FILTER] 👋 Shutdown complete.\n";
  return 0;
}
