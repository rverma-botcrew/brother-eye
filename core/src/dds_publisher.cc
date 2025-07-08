#include "dds_publisher.h"

#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace brother_eye {

// Helper function to set message header with current timestamp
template<typename MessageType>
void SetMessageHeader(MessageType& msg) {
  auto now = std::chrono::high_resolution_clock::now();
  auto time_since_epoch = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch - seconds);
  
  msg.header().stamp().sec(static_cast<int32_t>(seconds.count()));
  msg.header().stamp().nanosec(static_cast<uint32_t>(nanoseconds.count()));
  msg.header().frame_id("base_link");
}

// Helper function to safely write DDS messages with consistent error handling
template<typename WriterType, typename MessageType>
void SafeWrite(const std::shared_ptr<WriterType>& writer, const MessageType& msg, const std::string& msg_type) {
  try {
    writer->write(msg);
  } catch (const dds::core::Exception& e) {
    std::cerr << "[FILTER] ❌ DDS " << msg_type << " write exception: " << e.what() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "[FILTER] ❌ std::exception in " << msg_type << " write: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "[FILTER] ❌ Unknown exception in " << msg_type << " write\n";
  }
}

DdsPublisher::DdsPublisher() 
    : domain_participant_(0),
      input_topic_(domain_participant_, "dds_raw_points"),
      output_topic_(domain_participant_, "dds_clustered_points"),
      cluster_topic_(domain_participant_, "dds_cluster_analysis"),
      bbox_topic_(domain_participant_, "dds_bounding_boxes"),
      subscriber_(domain_participant_),
      publisher_(domain_participant_),
      reader_(nullptr),
      writer_(nullptr),
      cluster_writer_(nullptr),
      bbox_writer_(nullptr),
      initialized_(false) {
}

void DdsPublisher::Initialize() {
  auto reader_qos = dds::sub::qos::DataReaderQos()
    << dds::core::policy::Reliability::Reliable()
    << dds::core::policy::History::KeepLast(kHistoryDepth);

  auto writer_qos = dds::pub::qos::DataWriterQos()
    << dds::core::policy::Reliability::Reliable()
    << dds::core::policy::History::KeepLast(kHistoryDepth);

  reader_ = std::make_shared<dds::sub::DataReader<DDSPointCloud2>>(subscriber_, input_topic_, reader_qos);
  writer_ = std::make_shared<dds::pub::DataWriter<DDSPointCloud2>>(publisher_, output_topic_, writer_qos);
  cluster_writer_ = std::make_shared<dds::pub::DataWriter<DDSClusterArray>>(publisher_, cluster_topic_, writer_qos);
  bbox_writer_ = std::make_shared<dds::pub::DataWriter<DDSBoundingBoxArray>>(publisher_, bbox_topic_, writer_qos);
  
  initialized_ = true;

  std::cout << "[FILTER] 🔗 Subscribed to DDS topic: dds_raw_points\n";
  std::cout << "[FILTER] 📨 Publishing to DDS topic: dds_clustered_points\n";
  std::cout << "[FILTER] 📊 Publishing cluster analysis to DDS topic: dds_cluster_analysis\n";
  std::cout << "[FILTER] 📦 Publishing bounding boxes to DDS topic: dds_bounding_boxes\n";
}

void DdsPublisher::PublishPointCloud(const CloudT::Ptr& cloud, const DDSPointCloud2& prototype) {
  // Bail out if empty cloud
  if (cloud->points.empty()) {
    std::cerr << "[FILTER] ⚠️ Skipping publish: empty cloud\n";
    return;
  }

  // Bail out if prototype is invalid
  if (prototype.point_step() == 0) {
    std::cerr << "[FILTER] ❌ Invalid prototype: point_step=0\n";
    return;
  }

  DDSPointCloud2 output_msg = prototype;
  output_msg.width(cloud->points.size());
  output_msg.height(1);
  output_msg.row_step(output_msg.point_step() * output_msg.width());
  output_msg.is_dense(false);
  output_msg.data().resize(output_msg.row_step());

  // Copy point cloud data to DDS message
  uint8_t* data_ptr = output_msg.data().data();
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float* point_data = reinterpret_cast<float*>(data_ptr + i * output_msg.point_step());
    const auto& point = cloud->points[i];
    point_data[0] = point.x;
    point_data[1] = point.y;
    point_data[2] = point.z;
    point_data[3] = point.intensity;
  }

  SafeWrite(writer_, output_msg, "point cloud");
}

void DdsPublisher::PublishClusterAnalysis(const std::vector<ClusterInfo>& clusters,
                                         const std::map<int, TrackedObject>& tracked_objects) {
  auto cluster_array_msg = ConvertToClusterArray(clusters, tracked_objects);
  SafeWrite(cluster_writer_, cluster_array_msg, "cluster analysis");
  std::cout << "[FILTER] 📊 Published cluster analysis (" << cluster_array_msg.clusters().size() << " clusters)\n";
}

void DdsPublisher::PublishBoundingBoxes(const std::vector<ClusterInfo>& clusters) {
  auto bbox_array_msg = ConvertToBoundingBoxArray(clusters);
  SafeWrite(bbox_writer_, bbox_array_msg, "bounding boxes");
  std::cout << "[FILTER] 📦 Published bounding boxes (" << bbox_array_msg.boxes().size() << " boxes)\n";
}

DDSClusterArray DdsPublisher::ConvertToClusterArray(const std::vector<ClusterInfo>& clusters, 
                                                    const std::map<int, TrackedObject>& tracked_objects) {
  DDSClusterArray msg;
  
  // Set header with current time
  SetMessageHeader(msg);
  
  // Limit cluster count to avoid DDS sequence issues
  size_t cluster_count = std::min(tracked_objects.size(), kMaxClusters);
  
  std::cout << "[FILTER] 🔍 Converting " << tracked_objects.size() << " tracked objects to DDS (max " << kMaxClusters << ")\n";
  
  // Convert clusters to DDS format - first count valid ones
  std::vector<std::pair<int, TrackedObject>> valid_objects;
  for (const auto& [id, obj] : tracked_objects) {
    if (valid_objects.size() >= kMaxClusters) {
      std::cout << "[FILTER] ⚠️ Reached maximum cluster limit, stopping at " << kMaxClusters << "\n";
      break;
    }
    
    // Validate centroid data
    if (!ValidatePoint2D(obj.GetLastCentroid())) {
      std::cerr << "[FILTER] ⚠️ Invalid centroid for cluster " << id 
                << " [" << obj.GetLastCentroid().x << ", " << obj.GetLastCentroid().y << "], skipping\n";
      continue;
    }
    
    // Calculate distance and angle
    float distance = std::hypot(obj.GetLastCentroid().x, obj.GetLastCentroid().y);
    float angle = std::atan2(obj.GetLastCentroid().y, obj.GetLastCentroid().x);
    
    // Validate calculated values
    if (!IsValidFloat(distance) || !IsValidFloat(angle)) {
      std::cerr << "[FILTER] ⚠️ Invalid distance/angle for cluster " << id 
                << " [dist=" << distance << ", angle=" << angle << "], skipping\n";
      continue;
    }
    
    // Validate object age and lost_frames (should be non-negative)
    if (obj.GetAge() < 0 || obj.GetLostFrames() < 0) {
      std::cerr << "[FILTER] ⚠️ Invalid age/lost_frames for cluster " << id 
                << " [age=" << obj.GetAge() << ", lost=" << obj.GetLostFrames() << "], skipping\n";
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
  size_t cluster_index = 0;
  
  for (const auto& [tracked_id, tracked_obj] : valid_objects) {
    std::cout << "[FILTER] 🔍 Processing cluster " << tracked_id << " at index " << cluster_index << "\n";
    
    DDSClusterInfo& dds_cluster = msg.clusters()[cluster_index];
    
    // Set centroid with validation and clamping
    float centroid_x = tracked_obj.GetLastCentroid().x;
    float centroid_y = tracked_obj.GetLastCentroid().y;
    
    // Clamp coordinates to reasonable ranges
    centroid_x = std::max(kMinCoordinate, std::min(kMaxCoordinate, centroid_x));
    centroid_y = std::max(kMinCoordinate, std::min(kMaxCoordinate, centroid_y));
    
    dds_cluster.centroid().x(centroid_x);
    dds_cluster.centroid().y(centroid_y);
    
    // Calculate distance and angle with clamping
    float distance = std::hypot(centroid_x, centroid_y);
    float angle = std::atan2(centroid_y, centroid_x);
    
    // Clamp distance to reasonable range
    distance = std::max(0.0f, std::min(kMaxDistance, distance));
    
    dds_cluster.distance(distance);
    dds_cluster.angle(angle);
    dds_cluster.id(tracked_id);
    dds_cluster.age(std::max(0, tracked_obj.GetAge()));
    dds_cluster.lost_frames(std::max(0, tracked_obj.GetLostFrames()));
    
    // Set risk level based on distance thresholds
    uint8_t risk_level = 0;  // NONE
    if (distance < kRedRiskDistance) {
      risk_level = 3;  // RED
    } else if (distance < kYellowRiskDistance) {
      risk_level = 2;  // YELLOW
    } else if (distance < kGreenRiskDistance) {
      risk_level = 1;  // GREEN
    }
    dds_cluster.risk_level(risk_level);
    
    // Set default bounding box if no matching cluster found
    cv::Point3f bbox_center(centroid_x, centroid_y, kDefaultZHeight);
    cv::Point3f bbox_size(kDefaultBboxSize, kDefaultBboxSize, 1.0f);
    
    // Try to find matching cluster info for bounding box data
    for (const auto& cluster : clusters) {
      if (cv::norm(cluster.GetCentroid() - tracked_obj.GetLastCentroid()) < kClusterMatchDistance) {
        if (ValidatePoint3D(cluster.GetBoundingBoxCenter()) && ValidatePoint3D(cluster.GetBoundingBoxSize())) {
          bbox_center = cluster.GetBoundingBoxCenter();
          bbox_size = cluster.GetBoundingBoxSize();
        }
        break;
      }
    }
    
    // Clamp bounding box center to reasonable values
    bbox_center.x = std::max(kMinCoordinate, std::min(kMaxCoordinate, bbox_center.x));
    bbox_center.y = std::max(kMinCoordinate, std::min(kMaxCoordinate, bbox_center.y));
    bbox_center.z = std::max(kMinZCoordinate, std::min(kMaxZCoordinate, bbox_center.z));
    
    // Ensure reasonable bounding box size
    bbox_size.x = std::max(kMinBboxSize, std::min(kMaxBboxSize, bbox_size.x));
    bbox_size.y = std::max(kMinBboxSize, std::min(kMaxBboxSize, bbox_size.y));
    bbox_size.z = std::max(kMinBboxSize, std::min(kMaxBboxSize, bbox_size.z));
    
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
    
    std::cout << "[FILTER] 🔍 Cluster " << tracked_id << " data: centroid=[" << centroid_x << ", " << centroid_y 
              << "], dist=" << distance << ", bbox_size=[" << bbox_size.x << ", " << bbox_size.y << ", " << bbox_size.z << "]\n";
    
    ++cluster_index;
  }
  
  std::cout << "[FILTER] ✅ Converted " << cluster_index << " clusters to DDS message\n";
  return msg;
}

DDSBoundingBoxArray DdsPublisher::ConvertToBoundingBoxArray(const std::vector<ClusterInfo>& clusters) {
  DDSBoundingBoxArray bbox_array_msg;
  
  // Set header with current timestamp
  SetMessageHeader(bbox_array_msg);
  
  // Convert bounding boxes - first filter valid ones
  std::vector<ClusterInfo> valid_clusters;
  for (const auto& cluster : clusters) {
    if (ValidatePoint2D(cluster.GetCentroid())) {
      valid_clusters.push_back(cluster);
    } else {
      std::cerr << "[FILTER] ⚠️ Invalid cluster centroid, skipping from bbox array\n";
    }
  }
  
  bbox_array_msg.boxes().resize(valid_clusters.size());
  
  for (size_t cluster_index = 0; cluster_index < valid_clusters.size(); ++cluster_index) {
    const auto& cluster = valid_clusters[cluster_index];
    DDSBoundingBox3D& bounding_box = bbox_array_msg.boxes()[cluster_index];
    
    // Validate cluster bounding box data
    if (!ValidatePoint3D(cluster.GetBoundingBoxCenter()) || !ValidatePoint3D(cluster.GetBoundingBoxSize())) {
      std::cerr << "[FILTER] ⚠️ Invalid bounding box for cluster " << cluster_index << ", using defaults\n";
      // Set default center position
      bounding_box.center().position().x(cluster.GetCentroid().x);
      bounding_box.center().position().y(cluster.GetCentroid().y);
      bounding_box.center().position().z(kDefaultZHeight);
      
      // Set default size
      bounding_box.size().x(kDefaultBboxSize);
      bounding_box.size().y(kDefaultBboxSize);
      bounding_box.size().z(1.0f);
    } else {
      // Set center position
      bounding_box.center().position().x(cluster.GetBoundingBoxCenter().x);
      bounding_box.center().position().y(cluster.GetBoundingBoxCenter().y);
      bounding_box.center().position().z(cluster.GetBoundingBoxCenter().z);
      
      // Set size with minimum bounds
      bounding_box.size().x(std::max(kMinBboxSizeForArray, cluster.GetBoundingBoxSize().x));
      bounding_box.size().y(std::max(kMinBboxSizeForArray, cluster.GetBoundingBoxSize().y));
      bounding_box.size().z(std::max(kMinBboxSizeForArray, cluster.GetBoundingBoxSize().z));
    }
    
    // Set orientation (identity quaternion for axis-aligned boxes)
    bounding_box.center().orientation().x(0.0f);
    bounding_box.center().orientation().y(0.0f);
    bounding_box.center().orientation().z(0.0f);
    bounding_box.center().orientation().w(1.0f);
  }
  
  return bbox_array_msg;
}

bool DdsPublisher::IsValidFloat(float value) {
  return std::isfinite(value) && !std::isnan(value);
}

bool DdsPublisher::ValidatePoint2D(const cv::Point2f& point) {
  return IsValidFloat(point.x) && IsValidFloat(point.y);
}

bool DdsPublisher::ValidatePoint3D(const cv::Point3f& point) {
  return IsValidFloat(point.x) && IsValidFloat(point.y) && IsValidFloat(point.z);
}

}  // namespace brother_eye
