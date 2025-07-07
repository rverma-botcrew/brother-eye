#include <ddscxx/dds/dds.hpp>
#include "dds_pcl.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

class DDS2ROSBridge : public rclcpp::Node {
public:
  DDS2ROSBridge()
    : Node("dds_to_ros_pointcloud_bridge"),
      dp_(0),
      pointcloud_topic_(dp_, "dds_clustered_points"),
      cluster_topic_(dp_, "dds_cluster_analysis"),
      bbox_topic_(dp_, "dds_bounding_boxes"),
      sub_(dp_),
      pointcloud_reader_(sub_, pointcloud_topic_),
      cluster_reader_(sub_, cluster_topic_),
      bbox_reader_(sub_, bbox_topic_)
  {
    // ROS publishers
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ros_clustered_cloud", 10);
    cluster_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);
    bbox_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_box_markers", 10);

    // Timer to poll DDS topics
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DDS2ROSBridge::pollDDS, this));
  }


private:
  void pollDDS() {
    // Handle point cloud data
    pollPointCloud();
    
    // Handle cluster analysis data
    pollClusterAnalysis();
    
    // Handle bounding box data
    pollBoundingBoxes();
  }

  void pollPointCloud() {
    auto samples = pointcloud_reader_.take();
    for (auto &s : samples) {
      const auto &dds_msg = s.data();

      sensor_msgs::msg::PointCloud2 ros_msg;
      ros_msg.header.stamp = this->now();
      ros_msg.header.frame_id = dds_msg.header().frame_id();
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
      pointcloud_pub_->publish(ros_msg);

      RCLCPP_INFO(this->get_logger(), "Published point cloud with %d points", dds_msg.width());
    }
  }

  void pollClusterAnalysis() {
    auto samples = cluster_reader_.take();
    for (auto &s : samples) {
      const auto &dds_msg = s.data();
      
      visualization_msgs::msg::MarkerArray marker_array;
      
      for (size_t i = 0; i < dds_msg.clusters().size(); ++i) {
        const auto &cluster = dds_msg.clusters()[i];
        
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = dds_msg.header().frame_id();
        marker.ns = "clusters";
        marker.id = cluster.id();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at cluster centroid
        marker.pose.position.x = cluster.centroid().x();
        marker.pose.position.y = cluster.centroid().y();
        marker.pose.position.z = 0.5; // Fixed height for visibility
        marker.pose.orientation.w = 1.0;
        
        // Size based on risk level
        double scale = 0.3;
        if (cluster.risk_level() == 3) scale = 0.6; // RED
        else if (cluster.risk_level() == 2) scale = 0.5; // YELLOW
        else if (cluster.risk_level() == 1) scale = 0.4; // GREEN
        
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        
        // Color based on risk level
        marker.color.a = 0.8;
        if (cluster.risk_level() == 3) { // RED
          marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
        } else if (cluster.risk_level() == 2) { // YELLOW
          marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
        } else if (cluster.risk_level() == 1) { // GREEN
          marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        } else { // NONE
          marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5;
        }
        
        marker_array.markers.push_back(marker);
      }
      
      cluster_markers_pub_->publish(marker_array);
      RCLCPP_INFO(this->get_logger(), "Published %zu cluster markers", marker_array.markers.size());
    }
  }

  void pollBoundingBoxes() {
    auto samples = bbox_reader_.take();
    for (auto &s : samples) {
      const auto &dds_msg = s.data();
      
      visualization_msgs::msg::MarkerArray marker_array;
      
      for (size_t i = 0; i < dds_msg.boxes().size(); ++i) {
        const auto &bbox = dds_msg.boxes()[i];
        
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = dds_msg.header().frame_id();
        marker.ns = "bounding_boxes";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position and orientation
        marker.pose.position.x = bbox.center().position().x();
        marker.pose.position.y = bbox.center().position().y();
        marker.pose.position.z = bbox.center().position().z();
        marker.pose.orientation.x = bbox.center().orientation().x();
        marker.pose.orientation.y = bbox.center().orientation().y();
        marker.pose.orientation.z = bbox.center().orientation().z();
        marker.pose.orientation.w = bbox.center().orientation().w();
        
        // Size
        marker.scale.x = bbox.size().x();
        marker.scale.y = bbox.size().y();
        marker.scale.z = bbox.size().z();
        
        // Color (semi-transparent blue)
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.3;
        
        marker_array.markers.push_back(marker);
      }
      
      bbox_markers_pub_->publish(marker_array);
      RCLCPP_INFO(this->get_logger(), "Published %zu bounding box markers", marker_array.markers.size());
    }
  }

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // DDS components
  dds::domain::DomainParticipant dp_;
  dds::topic::Topic<pcl_dds_msgs::PointCloud2> pointcloud_topic_;
  dds::topic::Topic<pcl_dds_msgs::ClusterArray> cluster_topic_;
  dds::topic::Topic<pcl_dds_msgs::BoundingBoxArray> bbox_topic_;
  dds::sub::Subscriber sub_;
  dds::sub::DataReader<pcl_dds_msgs::PointCloud2> pointcloud_reader_;
  dds::sub::DataReader<pcl_dds_msgs::ClusterArray> cluster_reader_;
  dds::sub::DataReader<pcl_dds_msgs::BoundingBoxArray> bbox_reader_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDS2ROSBridge>());
  rclcpp::shutdown();
  return 0;
}
