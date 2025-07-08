#include "data_converter.hpp"
#include "constants.hpp"

#include <iostream>
#include <cstring>

namespace brother_eye {

CloudT::Ptr DataConverter::ConvertToPcl(const DDSPointCloud2& msg) {
  // Calculate point count from data size and point step (original method)
  if (msg.point_step() == DataConversionConstants::kZeroPointStep) {
    std::cerr << "[DATA_CONVERTER] ❌ Invalid point_step: 0\n";
    return CloudT::Ptr(new CloudT);
  }
  
  const size_t point_count = msg.data().size() / msg.point_step();
  CloudT::Ptr cloud(new CloudT);
  
  if (point_count == DataConversionConstants::kZeroPointCount) {
    std::cerr << "[DATA_CONVERTER] ⚠️ Empty message - no points to convert\n";
    return cloud;
  }
  
  std::cout << "[DATA_CONVERTER] 📊 Converting: data_size=" << msg.data().size() 
            << ", point_step=" << msg.point_step() 
            << ", calculated_points=" << point_count 
            << ", msg_width=" << msg.width() << "\n";
  
  // Initialize cloud properties
  cloud->width = static_cast<uint32_t>(point_count);
  cloud->height = DataConversionConstants::kPointCloudHeight;
  cloud->is_dense = false;
  cloud->points.resize(point_count);

  // Convert binary data to PCL points
  const uint8_t* raw_data = msg.data().data();
  for (size_t i = 0; i < point_count; ++i) {
    const float* point_data = reinterpret_cast<const float*>(raw_data + i * msg.point_step());
    auto& point = cloud->points[i];
    point.x = point_data[DataConversionConstants::kXFieldIndex];
    point.y = point_data[DataConversionConstants::kYFieldIndex];
    point.z = point_data[DataConversionConstants::kZFieldIndex];
    point.intensity = point_data[DataConversionConstants::kIntensityFieldIndex];
  }
  
  std::cout << "[DATA_CONVERTER] 📦 Successfully converted " << point_count << " points from DDS to PCL\n";
  return cloud;
}

DDSPointCloud2 DataConverter::ConvertToDds(const CloudT::Ptr& cloud, 
                                          const DDSPointCloud2& prototype) {
  DDSPointCloud2 output_msg = prototype;
  
  if (cloud->points.empty()) {
    std::cerr << "[DATA_CONVERTER] ⚠️ Empty cloud, returning empty message\n";
    output_msg.width(DataConversionConstants::kEmptyCloudWidth);
    output_msg.height(DataConversionConstants::kEmptyCloudHeight);
    output_msg.data().clear();
    return output_msg;
  }
  
  // Update message properties
  output_msg.width(cloud->points.size());
  output_msg.height(DataConversionConstants::kPointCloudHeight);
  output_msg.row_step(output_msg.point_step() * output_msg.width());
  output_msg.is_dense(cloud->is_dense);
  output_msg.data().resize(output_msg.row_step());

  // Copy point cloud data to DDS message
  uint8_t* data_ptr = output_msg.data().data();
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float* point_data = reinterpret_cast<float*>(data_ptr + i * output_msg.point_step());
    const auto& point = cloud->points[i];
    point_data[DataConversionConstants::kXFieldIndex] = point.x;
    point_data[DataConversionConstants::kYFieldIndex] = point.y;
    point_data[DataConversionConstants::kZFieldIndex] = point.z;
    point_data[DataConversionConstants::kIntensityFieldIndex] = point.intensity;
  }
  
  std::cout << "[DATA_CONVERTER] 📡 Converted " << cloud->points.size() << " points from PCL to DDS\n";
  return output_msg;
}

bool DataConverter::ValidateDdsMessage(const DDSPointCloud2& msg) {
  if (msg.point_step() == DataConversionConstants::kZeroPointStep) {
    std::cerr << "[DATA_CONVERTER] ❌ Invalid point_step: 0\n";
    return false;
  }
  
  if (msg.data().empty()) {
    std::cerr << "[DATA_CONVERTER] ⚠️ Empty data array\n";
    return false;
  }
  
  if (msg.width() == DataConversionConstants::kEmptyCloudWidth) {
    std::cerr << "[DATA_CONVERTER] ⚠️ Zero width message\n";
    return false;
  }
  
  return true;
}

size_t DataConverter::GetPointCount(const DDSPointCloud2& msg) {
  if (msg.point_step() == DataConversionConstants::kZeroPointStep) {
    return DataConversionConstants::kZeroPointCount;
  }
  return msg.data().size() / msg.point_step();  // Calculate from data size
}

}  // namespace brother_eye
