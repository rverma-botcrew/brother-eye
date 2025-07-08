#ifndef DATA_CONVERTER_HPP_
#define DATA_CONVERTER_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "dds_pcl.hpp"

namespace brother_eye {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using DDSPointCloud2 = pcl_dds_msgs::PointCloud2;

/**
 * @brief Data conversion utilities
 * 
 * Provides methods for converting between different point cloud formats
 * (DDS messages, PCL point clouds, etc.)
 */
class DataConverter {
 public:
  /**
   * @brief Converts DDS point cloud message to PCL format
   * @param msg DDS point cloud message
   * @return PCL point cloud
   */
  static CloudT::Ptr ConvertToPcl(const DDSPointCloud2& msg);
  
  /**
   * @brief Converts PCL point cloud to DDS message format
   * @param cloud PCL point cloud
   * @param prototype DDS message prototype for field information
   * @return DDS point cloud message
   */
  static DDSPointCloud2 ConvertToDds(const CloudT::Ptr& cloud, 
                                     const DDSPointCloud2& prototype);
  
  /**
   * @brief Validates DDS point cloud message format
   * @param msg DDS point cloud message to validate
   * @return True if message is valid, false otherwise
   */
  static bool ValidateDdsMessage(const DDSPointCloud2& msg);
  
  /**
   * @brief Gets point count from DDS message
   * @param msg DDS point cloud message
   * @return Number of points in the message
   */
  static size_t GetPointCount(const DDSPointCloud2& msg);
};

}  // namespace brother_eye

#endif  // DATA_CONVERTER_HPP_
