#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ddscxx/dds/dds.hpp>
#include "pcl.hpp"  // Make sure this includes generated CycloneDDS types

class PointCloudBridge : public rclcpp::Node {
public:
  PointCloudBridge()
    : Node("pointcloud_bridge") {
    
    RCLCPP_INFO(this->get_logger(), "Initializing DDS participant and writer");

    // CycloneDDS setup
    dds_participant_.emplace(0); // domain ID 0
    dds_topic_.emplace(*dds_participant_, "PointCloud");
    dds_publisher_.emplace(*dds_participant_);
    dds_writer_.emplace(*dds_publisher_, *dds_topic_);

    // ROS QoS config
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliable().keep_last(10);

    // ROS subscription to PointCloud2
    pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", qos,
      std::bind(&PointCloudBridge::pointcloud_callback, this, std::placeholders::_1)
    );

    // Timer to monitor publish frequency every 1 second
    freq_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (msg_count_ > 0) {
          auto now = std::chrono::steady_clock::now();
          double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_msg_time_).count() / 1000.0;

          if (elapsed > 0.0) {
            double freq = static_cast<double>(msg_count_) / elapsed;
            RCLCPP_INFO(this->get_logger(), "DDS publish frequency: %.2f Hz", freq);
          }

          // Reset for next interval
          msg_count_ = 0;
          last_msg_time_ = now;
        }
      }
    );

    RCLCPP_INFO(this->get_logger(), "PointCloudBridge initialized");
  }

  ~PointCloudBridge() {
    RCLCPP_INFO(this->get_logger(), "Cleaning up DDS resources");
    if (dds_writer_) dds_writer_->close();
    if (dds_publisher_) dds_publisher_->close();
    if (dds_topic_) dds_topic_->close();
    if (dds_participant_) dds_participant_->close();
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Frequency tracking
    if (msg_count_ == 0) {
      last_msg_time_ = std::chrono::steady_clock::now();
    }
    msg_count_++;

    // RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

    pcl_dds_msg::PointCloud2 dds_pointcloud;
    FillPointCloud2(*msg, dds_pointcloud);

    try {
      dds_writer_->write(dds_pointcloud);
      // RCLCPP_INFO(this->get_logger(), "Published to DDS successfully");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to DDS: %s", e.what());
    }
  }

  void FillPointCloud2(const sensor_msgs::msg::PointCloud2 &ros, pcl_dds_msg::PointCloud2 &dds) {
    // Header
    pcl_dds_msg::Header header;
    pcl_dds_msg::Time stamp;
    stamp.sec(ros.header.stamp.sec);
    stamp.nanosec(ros.header.stamp.nanosec);
    header.stamp(stamp);
    header.frame_id(ros.header.frame_id);
    dds.header(header);

    // Metadata
    dds.height(ros.height);
    dds.width(ros.width);
    dds.is_bigendian(ros.is_bigendian);
    dds.point_step(ros.point_step);
    dds.row_step(ros.row_step);
    dds.is_dense(ros.is_dense);

    // Fields
    std::vector<pcl_dds_msg::PointField> dds_fields;
    for (const auto &field : ros.fields) {
      pcl_dds_msg::PointField dds_field;
      dds_field.name(field.name);
      dds_field.offset(field.offset);
      dds_field.datatype(field.datatype);
      dds_field.count(field.count);
      dds_fields.push_back(dds_field);
    }
    dds.fields(dds_fields);

    // Data
    std::vector<uint8_t> data(ros.data.begin(), ros.data.end());
    dds.data(data);
  }

  // ROS subscription
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;

  // DDS components
  std::optional<dds::domain::DomainParticipant> dds_participant_;
  std::optional<dds::topic::Topic<pcl_dds_msg::PointCloud2>> dds_topic_;
  std::optional<dds::pub::Publisher> dds_publisher_;
  std::optional<dds::pub::DataWriter<pcl_dds_msg::PointCloud2>> dds_writer_;

  // Frequency monitor
  rclcpp::TimerBase::SharedPtr freq_timer_;
  std::chrono::steady_clock::time_point last_msg_time_;
  size_t msg_count_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
