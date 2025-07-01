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
 *   6. Publish cleaned cloud to DDS topic "filtered_points"
 */

#include <ddscxx/dds/dds.hpp>
#include "dds_pcl.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <csignal>
#include <thread>
#include <limits>

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;

constexpr int SECTOR_COUNT = 12;
constexpr float FULL_CIRCLE = 2.0f * M_PI;
constexpr float SECTOR_ANGLE = FULL_CIRCLE / SECTOR_COUNT;

static bool running = true;
void sigint_handler(int) { running = false; }

static CloudT::Ptr convertToPCL(const DDSPointCloud2 &msg) {
  if (msg.point_step() < sizeof(float) * 4 || msg.data().empty()) {
    std::cerr << "[FILTER] ⚠️ Invalid PointCloud2 message (empty or malformed)\n";
    return CloudT::Ptr(new CloudT);
  }

  const size_t point_count = msg.data().size() / msg.point_step();
  CloudT::Ptr cloud(new CloudT);
  cloud->width = static_cast<uint32_t>(point_count);
  cloud->height = 1;
  cloud->is_dense = false;  // Always mark false since filtering may introduce gaps
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

static CloudT::Ptr cleanCloud(const CloudT::Ptr &in) {
  std::cout << "[FILTER] 🔍 Pre-clean size: " << in->points.size() << std::endl;
  if (in->points.empty()) return CloudT::Ptr(new CloudT);

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(0.05f, 0.05f, 0.05f);
  CloudT::Ptr vg_cloud(new CloudT);
  vg.filter(*vg_cloud);

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(vg_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-2.0f, 10.0f);  // widened range
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
  sor.setStddevMulThresh(2.0f);  // less aggressive
  CloudT::Ptr cleaned(new CloudT);
  sor.filter(*cleaned);

  std::cout << "[FILTER] ✨ Post-clean size: " << cleaned->points.size() << std::endl;
  return cleaned;
}

static void publishCloud(const CloudT::Ptr &cloud,
                         dds::pub::DataWriter<DDSPointCloud2> &writer,
                         const DDSPointCloud2 &proto) {
  if (cloud->empty()) {
    std::cerr << "[FILTER] ⚠️ Skipping publish: empty cloud\n";
    return;
  }

  DDSPointCloud2 out = proto;
  out.width(cloud->points.size());
  out.height(1);
  out.row_step(out.point_step() * out.width());
  out.is_dense(false);  // Always false due to filtering

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
  writer.write(out);
}

enum class RiskLevel { NONE, GREEN, YELLOW, RED };

class SectorStatus {
public:
  float min_distance;
  RiskLevel risk;
  SectorStatus() : min_distance(std::numeric_limits<float>::max()), risk(RiskLevel::NONE) {}
  void update(float r) {
    min_distance = r;
    risk = r < 0.5f ? RiskLevel::RED :
           r < 1.2f ? RiskLevel::YELLOW :
                      RiskLevel::GREEN;
  }
};

void assessCollisionRisk(const CloudT::Ptr &cloud,
                         std::array<SectorStatus, SECTOR_COUNT> &sectors) {
  for (const auto &pt : cloud->points) {
    float r = std::hypot(pt.x, pt.y);
    if (r < 0.01f || r > 3.0f) continue;
    float angle = std::atan2(pt.y, pt.x);
    int sector = static_cast<int>(std::floor((angle + M_PI) / SECTOR_ANGLE)) % SECTOR_COUNT;
    if (r < sectors[sector].min_distance) {
      sectors[sector].update(r);
    }
  }
}

int main() {
  std::signal(SIGINT, sigint_handler);
  std::cout << "[FILTER] 🚀 Initializing PointCloud filter node...\n";

  dds::domain::DomainParticipant dp(0);
  dds::topic::Topic<DDSPointCloud2> in_topic(dp, "PointCloud");
  dds::topic::Topic<DDSPointCloud2> out_topic(dp, "filtered_points");
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

  std::cout << "[FILTER] 🔗 Subscribed to DDS topic: PointCloud\n";
  std::cout << "[FILTER] 📨 Publishing to DDS topic: filtered_points\n";

  size_t frame_count = 0;
  while (running) {
    auto samples = reader.take();
    if (samples.length() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    for (auto &s : samples) {
      const auto &msg = s.data();
      std::cout << "[FILTER] 📦 Received DDS frame #" << ++frame_count
                << " | Points = " << msg.data().size() / msg.point_step() << std::endl;

      auto pcl_cloud = convertToPCL(msg);
      auto cleaned   = cleanCloud(pcl_cloud);
      std::array<SectorStatus, SECTOR_COUNT> sectors;
      assessCollisionRisk(cleaned, sectors);

      for (int i = 0; i < sectors.size(); ++i) {
        std::string risk_str;
        switch (sectors[i].risk) {
          case RiskLevel::RED: risk_str = "RED"; break;
          case RiskLevel::YELLOW: risk_str = "YELLOW"; break;
          case RiskLevel::GREEN: risk_str = "GREEN"; break;
          default: risk_str = "NONE";
        }
        std::cout << "  ▸ Sector " << i << ": " << risk_str
                  << " (min_dist = " << sectors[i].min_distance << ")\n";
      }

      publishCloud(cleaned, writer, msg);
      std::cout << "[FILTER] ✅ Published cleaned cloud\n";
    }
  }

  std::cout << "[FILTER] 👋 Shutdown requested, exiting cleanly.\n";
  return 0;
}