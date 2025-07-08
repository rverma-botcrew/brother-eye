#include "json_exporter.hpp"
#include "constants.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <limits>
#include <cmath>

namespace brother_eye {

// JSON formatting constants
namespace JsonFormatting {
  constexpr const char* kIndent2 = "  ";
  constexpr const char* kIndent4 = "    ";
  constexpr const char* kIndent6 = "      ";
  constexpr const char* kIndent8 = "        ";
}

std::string JsonExporter::ClustersToJson(const std::vector<ClusterInfo>& clusters, 
                                        const std::map<int, TrackedObject>& tracked_objects,
                                        size_t frame_number) {
  std::ostringstream json;
  
  json << "{\n";
  json << GenerateTimestamp();
  json << JsonFormatting::kIndent2 << "\"frame_number\": " << frame_number << ",\n";
  json << JsonFormatting::kIndent2 << "\"frame_id\": \"base_link\",\n";
  json << JsonFormatting::kIndent2 << "\"cluster_count\": " << clusters.size() << ",\n";
  json << JsonFormatting::kIndent2 << "\"tracked_object_count\": " << tracked_objects.size() << ",\n";
  json << GenerateStatistics(clusters);
  json << JsonFormatting::kIndent2 << "\"clusters\": [\n";
  
  // Add cluster data
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& cluster = clusters[i];
    
    // Find matching tracked object
    int tracked_id = JsonConstants::kInvalidTrackingId;
    int age = JsonConstants::kInitialAge;
    int lost_frames = JsonConstants::kInitialLostFrames;
    
    for (const auto& [id, obj] : tracked_objects) {
      if (cv::norm(cluster.GetCentroid() - obj.GetLastCentroid()) < JsonConstants::kClusterMatchThreshold) {
        tracked_id = id;
        age = obj.GetAge();
        lost_frames = obj.GetLostFrames();
        break;
      }
    }
    
    json << ClusterInfoToJson(cluster, tracked_id, age, lost_frames);
    if (i < clusters.size() - 1) {
      json << ",";
    }
    json << "\n";
  }
  
  json << JsonFormatting::kIndent2 << "]\n";
  json << "}";
  
  return json.str();
}

void JsonExporter::PublishJsonToFile(const std::string& json_data, const std::string& filename) {
  try {
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "[JSON_EXPORTER] ❌ Failed to open file: " << filename << "\n";
      return;
    }
    
    file << json_data;
    file.close();
    std::cout << "[JSON_EXPORTER] 📄 Successfully published JSON to " << filename << "\n";
    
  } catch (const std::exception& e) {
    std::cerr << "[JSON_EXPORTER] ❌ File write error: " << e.what() << "\n";
  }
}

void JsonExporter::PublishJsonToConsole(const std::string& json_data) {
  std::cout << "[JSON_EXPORTER] 📄 JSON Output:\n" << json_data << "\n" << std::endl;
}

std::string JsonExporter::EscapeJsonString(const std::string& input) {
  std::ostringstream escaped;
  
  for (char c : input) {
    switch (c) {
      case '"':  escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b";  break;
      case '\f': escaped << "\\f";  break;
      case '\n': escaped << "\\n";  break;
      case '\r': escaped << "\\r";  break;
      case '\t': escaped << "\\t";  break;
      default:   escaped << c;      break;
    }
  }
  
  return escaped.str();
}

std::string JsonExporter::FormatFloat(float value, int precision) {
  std::ostringstream formatted;
  formatted << std::fixed << std::setprecision(precision) << value;
  return formatted.str();
}

std::string JsonExporter::ClusterInfoToJson(const ClusterInfo& cluster, int tracked_id, int age, int lost_frames) {
  std::ostringstream json;
  
  json << JsonFormatting::kIndent4 << "{\n";
  
  // Basic cluster information
  json << JsonFormatting::kIndent6 << "\"id\": " << cluster.GetClusterId() << ",\n";
  json << JsonFormatting::kIndent6 << "\"tracked_id\": " << tracked_id << ",\n";
  
  // Centroid position
  json << JsonFormatting::kIndent6 << "\"centroid\": {\n";
  json << JsonFormatting::kIndent8 << "\"x\": " << FormatFloat(cluster.GetCentroid().x) << ",\n";
  json << JsonFormatting::kIndent8 << "\"y\": " << FormatFloat(cluster.GetCentroid().y) << "\n";
  json << JsonFormatting::kIndent6 << "},\n";
  
  // Distance and angle information
  json << JsonFormatting::kIndent6 << "\"distance\": " << FormatFloat(cluster.GetDistance()) << ",\n";
  json << JsonFormatting::kIndent6 << "\"angle_radians\": " << FormatFloat(cluster.GetAngle()) << ",\n";
  json << JsonFormatting::kIndent6 << "\"angle_degrees\": " << FormatFloat(cluster.GetAngle() * 180.0f / M_PI) << ",\n";
  
  // Risk assessment
  json << GenerateRiskJson(cluster);
  
  // Bounding box information
  json << GenerateBoundingBoxJson(cluster);
  
  // Tracking information
  json << JsonFormatting::kIndent6 << "\"tracking\": {\n";
  json << JsonFormatting::kIndent8 << "\"age\": " << age << ",\n";
  json << JsonFormatting::kIndent8 << "\"lost_frames\": " << lost_frames << "\n";
  json << JsonFormatting::kIndent6 << "}\n";
  
  json << JsonFormatting::kIndent4 << "}";
  
  return json.str();
}

std::string JsonExporter::GenerateTimestamp() {
  const auto now = std::chrono::high_resolution_clock::now();
  const auto time_since_epoch = now.time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
  
  std::ostringstream timestamp;
  timestamp << JsonFormatting::kIndent2 << "\"timestamp\": {\n";
  timestamp << JsonFormatting::kIndent4 << "\"seconds\": " << seconds.count() << ",\n";
  timestamp << JsonFormatting::kIndent4 << "\"milliseconds\": " << milliseconds.count() << ",\n";
  timestamp << JsonFormatting::kIndent4 << "\"iso8601\": \"" << std::to_string(seconds.count()) << "." 
            << std::setfill('0') << std::setw(3) << milliseconds.count() << "\"\n";
  timestamp << JsonFormatting::kIndent2 << "},\n";
  
  return timestamp.str();
}

std::string JsonExporter::GenerateStatistics(const std::vector<ClusterInfo>& clusters) {
  if (clusters.empty()) {
    return GenerateEmptyStatistics();
  }
  
  // Initialize counters and distance trackers
  size_t red_count = 0, yellow_count = 0, green_count = 0, none_count = 0;
  float min_distance = std::numeric_limits<float>::max();
  float max_distance = 0.0f;
  
  // Analyze each cluster
  for (const auto& cluster : clusters) {
    // Count risk levels
    switch (cluster.GetRiskLevel()) {
      case RiskLevel::kRed:    red_count++;    break;
      case RiskLevel::kYellow: yellow_count++; break;
      case RiskLevel::kGreen:  green_count++;  break;
      default:                 none_count++;   break;
    }
    
    // Track distance range
    const float distance = cluster.GetDistance();
    min_distance = std::min(min_distance, distance);
    max_distance = std::max(max_distance, distance);
  }
  
  // Generate statistics JSON
  std::ostringstream stats;
  stats << JsonFormatting::kIndent2 << "\"statistics\": {\n";
  stats << JsonFormatting::kIndent4 << "\"risk_distribution\": {\n";
  stats << JsonFormatting::kIndent6 << "\"red\": " << red_count << ",\n";
  stats << JsonFormatting::kIndent6 << "\"yellow\": " << yellow_count << ",\n";
  stats << JsonFormatting::kIndent6 << "\"green\": " << green_count << ",\n";
  stats << JsonFormatting::kIndent6 << "\"none\": " << none_count << "\n";
  stats << JsonFormatting::kIndent4 << "},\n";
  stats << JsonFormatting::kIndent4 << "\"distance_range\": {\n";
  stats << JsonFormatting::kIndent6 << "\"min\": " << FormatFloat(min_distance) << ",\n";
  stats << JsonFormatting::kIndent6 << "\"max\": " << FormatFloat(max_distance) << "\n";
  stats << JsonFormatting::kIndent4 << "}\n";
  stats << JsonFormatting::kIndent2 << "},\n";
  
  return stats.str();
}

std::string JsonExporter::GenerateEmptyStatistics() {
  std::ostringstream stats;
  stats << JsonFormatting::kIndent2 << "\"statistics\": {\n";
  stats << JsonFormatting::kIndent4 << "\"risk_distribution\": {\n";
  stats << JsonFormatting::kIndent6 << "\"red\": 0,\n";
  stats << JsonFormatting::kIndent6 << "\"yellow\": 0,\n";
  stats << JsonFormatting::kIndent6 << "\"green\": 0,\n";
  stats << JsonFormatting::kIndent6 << "\"none\": 0\n";
  stats << JsonFormatting::kIndent4 << "},\n";
  stats << JsonFormatting::kIndent4 << "\"distance_range\": {\n";
  stats << JsonFormatting::kIndent6 << "\"min\": 0.0,\n";
  stats << JsonFormatting::kIndent6 << "\"max\": 0.0\n";
  stats << JsonFormatting::kIndent4 << "}\n";
  stats << JsonFormatting::kIndent2 << "},\n";
  
  return stats.str();
}

std::string JsonExporter::GenerateRiskJson(const ClusterInfo& cluster) {
  std::ostringstream json;
  
  // Risk level as both enum and string
  std::string risk_str;
  int risk_level = 0;
  
  switch (cluster.GetRiskLevel()) {
    case RiskLevel::kRed: 
      risk_str = "RED"; 
      risk_level = 3; 
      break;
    case RiskLevel::kYellow: 
      risk_str = "YELLOW"; 
      risk_level = 2; 
      break;
    case RiskLevel::kGreen: 
      risk_str = "GREEN"; 
      risk_level = 1; 
      break;
    default: 
      risk_str = "NONE"; 
      risk_level = 0; 
      break;
  }
  
  json << JsonFormatting::kIndent6 << "\"risk\": {\n";
  json << JsonFormatting::kIndent8 << "\"level\": " << risk_level << ",\n";
  json << JsonFormatting::kIndent8 << "\"name\": \"" << risk_str << "\"\n";
  json << JsonFormatting::kIndent6 << "},\n";
  
  return json.str();
}

std::string JsonExporter::GenerateBoundingBoxJson(const ClusterInfo& cluster) {
  std::ostringstream json;
  
  const cv::Point3f& center = cluster.GetBoundingBoxCenter();
  const cv::Point3f& size = cluster.GetBoundingBoxSize();
  
  json << JsonFormatting::kIndent6 << "\"bounding_box\": {\n";
  
  // Center coordinates
  json << JsonFormatting::kIndent8 << "\"center\": {\n";
  json << JsonFormatting::kIndent8 << "  \"x\": " << FormatFloat(center.x) << ",\n";
  json << JsonFormatting::kIndent8 << "  \"y\": " << FormatFloat(center.y) << ",\n";
  json << JsonFormatting::kIndent8 << "  \"z\": " << FormatFloat(center.z) << "\n";
  json << JsonFormatting::kIndent8 << "},\n";
  
  // Size dimensions
  json << JsonFormatting::kIndent8 << "\"size\": {\n";
  json << JsonFormatting::kIndent8 << "  \"width\": " << FormatFloat(size.x) << ",\n";
  json << JsonFormatting::kIndent8 << "  \"height\": " << FormatFloat(size.y) << ",\n";
  json << JsonFormatting::kIndent8 << "  \"depth\": " << FormatFloat(size.z) << "\n";
  json << JsonFormatting::kIndent8 << "},\n";
  
  // Volume calculation
  const float volume = size.x * size.y * size.z;
  json << JsonFormatting::kIndent8 << "\"volume\": " << FormatFloat(volume) << "\n";
  
  json << JsonFormatting::kIndent6 << "},\n";
  
  return json.str();
}

}  // namespace brother_eye
