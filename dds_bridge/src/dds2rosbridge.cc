#include <ddscxx/dds/dds.hpp>
#include "dds_pcl.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class DDS2ROSBridge : public rclcpp::Node {
public:
  DDS2ROSBridge()
    : Node("dds_to_ros_pointcloud_bridge"),
      dp_(0),
      topic_(dp_, "dds_clustered_points"),
      sub_(dp_),
      reader_(sub_, topic_)  // ✅ Initialize in constructor!
  {
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ros_clustered_cloud", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DDS2ROSBridge::pollDDS, this));
  }


private:
  void pollDDS() {
    auto samples = reader_.take();
    for (auto &s : samples) {
      const auto &dds_msg = s.data();

      sensor_msgs::msg::PointCloud2 ros_msg;
      ros_msg.header.stamp = this->now();
      ros_msg.header.frame_id = dds_msg.header().frame_id();  // <-- Update if you use something else
      ros_msg.height = dds_msg.height();
      ros_msg.width = dds_msg.width();
      ros_msg.is_dense = dds_msg.is_dense();
      ros_msg.is_bigendian = dds_msg.is_bigendian();
      ros_msg.point_step = dds_msg.point_step();
      ros_msg.row_step = dds_msg.row_step();

      // Define point fields: x, y, z, intensity
      ros_msg.fields.resize(4);
      ros_msg.fields[0].name = "x";
      ros_msg.fields[0].offset = 0;
      ros_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      ros_msg.fields[0].count = 1;
      ros_msg.fields[1].name = "y";
      ros_msg.fields[1].offset = 4;
      ros_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      ros_msg.fields[1].count = 1;
      ros_msg.fields[2].name = "z";
      ros_msg.fields[2].offset = 8;
      ros_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      ros_msg.fields[2].count = 1;
      ros_msg.fields[3].name = "intensity";
      ros_msg.fields[3].offset = 12;
      ros_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
      ros_msg.fields[3].count = 1;

      // Copy data
      ros_msg.data = dds_msg.data();
      pub_->publish(ros_msg);

      RCLCPP_INFO(this->get_logger(), "Recieved %d DDS Samples", samples.length());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  dds::domain::DomainParticipant dp_;
  dds::topic::Topic<pcl_dds_msgs::PointCloud2> topic_;
  dds::sub::Subscriber sub_;
  dds::sub::DataReader<pcl_dds_msgs::PointCloud2> reader_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDS2ROSBridge>());
  rclcpp::shutdown();
  return 0;
}
