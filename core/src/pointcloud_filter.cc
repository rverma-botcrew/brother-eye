/*
 * pointcloud_filter.cpp
 * --------------------
 * Minimal ROS‑free point‑cloud cleaning pipeline for Velodyne → CycloneDDS streams.
 *
 * Build (Ubuntu 22.04):
 *   sudo apt install libcyclonedds-cpp-dev libpcl-dev
 *   mkdir build && cd build && cmake .. && make -j$(nproc)
 *
 * Requires CMakeLists.txt (see bottom comment) + IDL‑generated header "PointCloud_IDL.hpp".
 *
 * Pipeline:
 *   1. Convert DDS PointCloud2 → pcl::PointCloud<pcl::PointXYZI>
 *   2. VoxelGrid (leaf 5 cm)
 *   3. PassThrough Z∈[0.5,5] m
 *   4. RANSAC plane → remove ground
 *   5. Statistical Outlier Removal
 *   6. Publish cleaned cloud to DDS topic "dds_clustered_points"
 */

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

// --- Aliases ---
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;

// --- Constants ---
constexpr int SECTOR_COUNT = 12;
constexpr float FULL_CIRCLE = 2.0f * M_PI;
constexpr float SECTOR_ANGLE = FULL_CIRCLE / SECTOR_COUNT;
constexpr int RISK_HOLD_FRAMES = 5;

// --- Risk Level Enum ---
enum class RiskLevel { NONE, GREEN, YELLOW, RED };

// --- Sector Memory for Temporal Smoothing ---
class SectorMemory {
public:
  int hold_count = 0;
  RiskLevel last_risk = RiskLevel::NONE;

  void update(RiskLevel new_risk) {
    if (new_risk > last_risk) {
      last_risk = new_risk;
      hold_count = RISK_HOLD_FRAMES;
    } else if (hold_count > 0) {
      --hold_count;
    } else {
      last_risk = new_risk;
    }
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
    setIdentity(kf.processNoiseCov, cv::Scalar(2e-1));
    setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));
    // setIdentity(kf.processNoiseCov, cv::Scalar(5e-2));     // Prediction Trust = more smoothing
    // setIdentity(kf.measurementNoiseCov, cv::Scalar(5e-2)); // Measurement trust = bit more trust in sensor
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


// --- Sector Status (Per Frame) ---
class SectorStatus {
public:
  float min_distance;
  RiskLevel risk;
  SectorStatus() : min_distance(std::numeric_limits<float>::max()), risk(RiskLevel::NONE) {}
  void update(float r) {
    min_distance = r;
    risk = r < 1.5f ? RiskLevel::RED :
           r < 2.0f ? RiskLevel::YELLOW :
                      RiskLevel::GREEN;
  }
};


// --- Globals ---
std::array<SectorMemory, SECTOR_COUNT> sector_memory;
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


// --- Risk Assessment ---
void assessCollisionRisk(const CloudT::Ptr &cloud,
                         std::array<SectorStatus, SECTOR_COUNT> &sectors) {
  for (const auto &pt : cloud->points) {
    float r = std::hypot(pt.x, pt.y);
    if (r < 0.9f || r > 90.0f) continue;
    float angle = std::atan2(pt.y, pt.x);
    int sector = std::min(SECTOR_COUNT - 1, std::max(0, static_cast<int>((angle + M_PI) / SECTOR_ANGLE)));
    if (r < sectors[sector].min_distance) {
      sectors[sector].update(r);
    }
  }

  for (int i = 0; i < SECTOR_COUNT; ++i) {
    sector_memory[i].update(sectors[i].risk);
    sectors[i].risk = sector_memory[i].last_risk;
  }
}

// --- DDS Publisher ---
// static void publishCloud(const CloudT::Ptr &cloud,
//                          dds::pub::DataWriter<DDSPointCloud2> &writer,
//                          const DDSPointCloud2 &proto) {
//   DDSPointCloud2 out = proto;
//   out.width(cloud->points.size());
//   out.height(1);
//   out.row_step(out.point_step() * out.width());
//   out.is_dense(false);
//   out.data().resize(out.row_step());

//   uint8_t *dst = out.data().data();
//   for (size_t i = 0; i < cloud->points.size(); ++i) {
//     float *p_dst = reinterpret_cast<float *>(dst + i * out.point_step());
//     const auto &p = cloud->points[i];
//     p_dst[0] = p.x;
//     p_dst[1] = p.y;
//     p_dst[2] = p.z;
//     p_dst[3] = p.intensity;
//   }
//   writer.write(out);
// }

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


// --- Visual UI (optional) ---
void displayRiskUI(const std::array<SectorStatus, SECTOR_COUNT> &sectors) {
  const int radius = 200;
  const cv::Point center(radius + 10, radius + 10);
  const int img_size = 2 * radius + 20;

  cv::Mat img(img_size, img_size, CV_8UC3, cv::Scalar(30, 30, 30));

  for (int i = 0; i < SECTOR_COUNT; ++i) {
    float angle1 = i * SECTOR_ANGLE;
    float angle2 = (i + 1) * SECTOR_ANGLE;

    cv::Scalar color;
    switch (sectors[i].risk) {
      case RiskLevel::RED: color = cv::Scalar(0, 0, 255); break;
      case RiskLevel::YELLOW: color = cv::Scalar(0, 255, 255); break;
      case RiskLevel::GREEN: color = cv::Scalar(0, 255, 0); break;
      default: color = cv::Scalar(100, 100, 100);
    }

    cv::ellipse(img, center, cv::Size(radius, radius),
                0, angle1 * 180 / M_PI, angle2 * 180 / M_PI, color, -1);
  }

  cv::circle(img, center, radius, cv::Scalar(255, 255, 255), 2);
  for (int i = 0; i < SECTOR_COUNT; ++i) {
    float a = i * SECTOR_ANGLE;
    cv::Point end(center.x + radius * std::cos(a),
                  center.y + radius * std::sin(a));
    cv::line(img, center, end, cv::Scalar(255, 255, 255), 1);
  }

  cv::imshow("Collision Risk", img);
  cv::waitKey(1);
}

// --- Main ---
int main() {
  std::signal(SIGINT, sigint_handler);
  std::cout << "[FILTER] 🚀 Initializing PointCloud filter node...\n";

  dds::domain::DomainParticipant dp(0);
  dds::topic::Topic<DDSPointCloud2> in_topic(dp, "dds_raw_points");
  dds::topic::Topic<DDSPointCloud2> out_topic(dp, "dds_clustered_points");
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

  std::cout << "[FILTER] 🔗 Subscribed to DDS topic: dds_raw_points\n";
  std::cout << "[FILTER] 📨 Publishing to DDS topic: dds_clustered_points\n";

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
      auto centroids = extractCentroids(cleaned);

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

// Predict for unmatched tracked objects
for (auto& [id, obj] : tracked_objects) {
  if (matched_ids.count(id) == 0) {
    obj.predict();
  }
}

// Optionally: clean up old trackers not updated
for (auto it = tracked_objects.begin(); it != tracked_objects.end();) {
  if (it->second.lost_frames > 10) {
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


      std::array<SectorStatus, SECTOR_COUNT> sectors;
      assessCollisionRisk(cleaned, sectors);
      displayRiskUI(sectors);

      try {
        publishCloud(clustered, writer, msg);
        std::cout << "[FILTER] ✅ Published clustered cloud\n";
      } catch (const dds::core::Exception &e) {
        std::cerr << "[FILTER] ❌ publishCloud exception: " << e.what() << "\n";
      } catch (const std::exception &e) {
        std::cerr << "[FILTER] ❌ publishCloud std::exception: " << e.what() << "\n";
      } catch (...) {
        std::cerr << "[FILTER] ❌ publishCloud unknown exception\n";
      }
    }
  }

  std::cout << "[FILTER] 👋 Shutdown complete.\n";
  return 0;
}
