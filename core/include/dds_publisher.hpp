#ifndef DDS_PUBLISHER_H_
#define DDS_PUBLISHER_H_

#include <ddscxx/dds/dds.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <memory>

#include "cluster_info.hpp"
#include "kalman_tracker.hpp"
#include "dds_pcl.hpp"

namespace brother_eye {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;
using DDSClusterArray = pcl_dds_msgs::ClusterArray;
using DDSClusterInfo = pcl_dds_msgs::ClusterInfo;
using DDSBoundingBoxArray = pcl_dds_msgs::BoundingBoxArray;
using DDSBoundingBox3D = pcl_dds_msgs::BoundingBox3D;

class DdsPublisher {
 public:
  DdsPublisher();
  ~DdsPublisher() = default;
  
  // Initialize DDS components
  void Initialize();
  
  // Publishing functions
  void PublishPointCloud(const CloudT::Ptr& cloud, const DDSPointCloud2& prototype);
  void PublishClusterAnalysis(const std::vector<ClusterInfo>& clusters,
                             const std::map<int, TrackedObject>& tracked_objects);
  void PublishBoundingBoxes(const std::vector<ClusterInfo>& clusters);
  
  // Getter for reader
  std::shared_ptr<dds::sub::DataReader<DDSPointCloud2>> GetReader() { return reader_; }
  
 private:
  // DDS components
  dds::domain::DomainParticipant domain_participant_;
  dds::topic::Topic<DDSPointCloud2> input_topic_;
  dds::topic::Topic<DDSPointCloud2> output_topic_;
  dds::topic::Topic<DDSClusterArray> cluster_topic_;
  dds::topic::Topic<DDSBoundingBoxArray> bbox_topic_;
  dds::sub::Subscriber subscriber_;
  dds::pub::Publisher publisher_;
  
  // Note: These will be initialized in Initialize() method
  std::shared_ptr<dds::sub::DataReader<DDSPointCloud2>> reader_;
  std::shared_ptr<dds::pub::DataWriter<DDSPointCloud2>> writer_;
  std::shared_ptr<dds::pub::DataWriter<DDSClusterArray>> cluster_writer_;
  std::shared_ptr<dds::pub::DataWriter<DDSBoundingBoxArray>> bbox_writer_;
  
  bool initialized_ = false;
  
  // Conversion functions
  DDSClusterArray ConvertToClusterArray(const std::vector<ClusterInfo>& clusters, 
                                       const std::map<int, TrackedObject>& tracked_objects);
  DDSBoundingBoxArray ConvertToBoundingBoxArray(const std::vector<ClusterInfo>& clusters);
  
  // Validation functions
  bool IsValidFloat(float value);
  bool ValidatePoint2D(const cv::Point2f& point);
  bool ValidatePoint3D(const cv::Point3f& point);
  
  // Configuration constants
  static constexpr size_t kMaxClusters = 512;
  static constexpr int kHistoryDepth = 10;
  
  // Distance and validation thresholds
  static constexpr float kClusterMatchDistance = 0.3f;
  static constexpr float kRedRiskDistance = 1.5f;
  static constexpr float kYellowRiskDistance = 2.0f;
  static constexpr float kGreenRiskDistance = 5.0f;
  
  // Clamping limits
  static constexpr float kMaxCoordinate = 100.0f;
  static constexpr float kMinCoordinate = -100.0f;
  static constexpr float kMaxDistance = 200.0f;
  static constexpr float kMaxZCoordinate = 10.0f;
  static constexpr float kMinZCoordinate = -10.0f;
  static constexpr float kMinBboxSize = 0.01f;
  static constexpr float kMaxBboxSize = 50.0f;
  static constexpr float kDefaultBboxSize = 0.5f;
  static constexpr float kDefaultZHeight = 0.5f;
  static constexpr float kMinBboxSizeForArray = 0.1f;
};

}  // namespace brother_eye

#endif  // DDS_PUBLISHER_H_
