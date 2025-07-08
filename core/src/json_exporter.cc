#include "json_exporter.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <limits>
#include <cmath>

namespace brother_eye {

std::string JsonExporter::ClustersToJson(const std::vector<ClusterInfo>& clusters, 
                                        const std::map<int, TrackedObject>& tracked_objects,
                                        size_t frame_number) {
  std::ostringstream json;
  
  json << "{\n";
  json << GenerateTimestamp();
  json << "  \"frame_number\": " << frame_number << ",\n";
  json << "  \"frame_id\": \"base_link\",\n";
  json << "  \"cluster_count\": " << clusters.size() << ",\n";
  json << "  \"tracked_object_count\": " << tracked_objects.size() << ",\n";
  json << GenerateStatistics(clusters);
  json << "  \"clusters\": [\n";
  
  // Add cluster data
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& cluster = clusters[i];
    
    // Find matching tracked object
    int tracked_id = -1;
    int age = 0;
    int lost_frames = 0;
    
    for (const auto& [id, obj] : tracked_objects) {
      if (cv::norm(cluster.GetCentroid() - obj.GetLastCentroid()) < 0.3f) {
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
  
  json << "  ]\n";
  json << "}";
  
  return json.str();
}

void JsonExporter::PublishJsonToFile(const std::string& json_data, const std::string& filename) {
  try {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << json_data;
      file.close();
      std::cout << "[FILTER] 📄 Published JSON to " << filename << "\n";
    } else {
      std::cerr << "[FILTER] ❌ Failed to open JSON file: " << filename << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "[FILTER] ❌ JSON file write error: " << e.what() << "\n";
  }
}

void JsonExporter::PublishJsonToConsole(const std::string& json_data) {
  std::cout << "[FILTER] 📄 JSON Output:\n" << json_data << "\n" << std::endl;
}

std::string JsonExporter::EscapeJsonString(const std::string& input) {
  std::ostringstream ss;
  for (char c : input) {
    switch (c) {
      case '"': ss << "\\\""; break;
      case '\\': ss << "\\\\"; break;
      case '\b': ss << "\\b"; break;
      case '\f': ss << "\\f"; break;
      case '\n': ss << "\\n"; break;
      case '\r': ss << "\\r"; break;
      case '\t': ss << "\\t"; break;
      default: ss << c; break;
    }
  }
  return ss.str();
}

std::string JsonExporter::FormatFloat(float value, int precision) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  return ss.str();
}

std::string JsonExporter::ClusterInfoToJson(const ClusterInfo& cluster, int tracked_id, int age, int lost_frames) {
  std::ostringstream json;
  json << "    {\n";
  json << "      \"id\": " << cluster.GetId() << ",\n";
  json << "      \"tracked_id\": " << tracked_id << ",\n";
  json << "      \"centroid\": {\n";
  json << "        \"x\": " << FormatFloat(cluster.GetCentroid().x) << ",\n";
  json << "        \"y\": " << FormatFloat(cluster.GetCentroid().y) << "\n";
  json << "      },\n";
  json << "      \"distance\": " << FormatFloat(cluster.GetDistance()) << ",\n";
  json << "      \"angle_radians\": " << FormatFloat(cluster.GetAngle()) << ",\n";
  json << "      \"angle_degrees\": " << FormatFloat(cluster.GetAngle() * 180.0f / M_PI) << ",\n";
  
  // Risk level as both enum and string
  std::string risk_str;
  int risk_level = 0;
  switch (cluster.GetRisk()) {
    case RiskLevel::kRed: risk_str = "RED"; risk_level = 3; break;
    case RiskLevel::kYellow: risk_str = "YELLOW"; risk_level = 2; break;
    case RiskLevel::kGreen: risk_str = "GREEN"; risk_level = 1; break;
    default: risk_str = "NONE"; risk_level = 0; break;
  }
  
  json << "      \"risk\": {\n";
  json << "        \"level\": " << risk_level << ",\n";
  json << "        \"name\": \"" << risk_str << "\"\n";
  json << "      },\n";
  
  // Bounding box
  json << "      \"bounding_box\": {\n";
  json << "        \"center\": {\n";
  json << "          \"x\": " << FormatFloat(cluster.GetBboxCenter().x) << ",\n";
  json << "          \"y\": " << FormatFloat(cluster.GetBboxCenter().y) << ",\n";
  json << "          \"z\": " << FormatFloat(cluster.GetBboxCenter().z) << "\n";
  json << "        },\n";
  json << "        \"size\": {\n";
  json << "          \"width\": " << FormatFloat(cluster.GetBboxSize().x) << ",\n";
  json << "          \"height\": " << FormatFloat(cluster.GetBboxSize().y) << ",\n";
  json << "          \"depth\": " << FormatFloat(cluster.GetBboxSize().z) << "\n";
  json << "        },\n";
  json << "        \"volume\": " << FormatFloat(cluster.GetBboxSize().x * cluster.GetBboxSize().y * cluster.GetBboxSize().z) << "\n";
  json << "      },\n";
  
  // Tracking information
  json << "      \"tracking\": {\n";
  json << "        \"age\": " << age << ",\n";
  json << "        \"lost_frames\": " << lost_frames << "\n";
  json << "      }\n";
  json << "    }";
  
  return json.str();
}

std::string JsonExporter::GenerateTimestamp() {
  auto now = std::chrono::high_resolution_clock::now();
  auto time_since_epoch = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
  
  std::ostringstream timestamp;
  timestamp << "  \"timestamp\": {\n";
  timestamp << "    \"seconds\": " << seconds.count() << ",\n";
  timestamp << "    \"milliseconds\": " << milliseconds.count() << ",\n";
  timestamp << "    \"iso8601\": \"" << std::to_string(seconds.count()) << "." 
            << std::setfill('0') << std::setw(3) << milliseconds.count() << "\"\n";
  timestamp << "  },\n";
  
  return timestamp.str();
}

std::string JsonExporter::GenerateStatistics(const std::vector<ClusterInfo>& clusters) {
  // Statistics
  size_t red_count = 0, yellow_count = 0, green_count = 0, none_count = 0;
  float min_distance = std::numeric_limits<float>::max();
  float max_distance = 0.0f;
  
  for (const auto& cluster : clusters) {
    switch (cluster.GetRisk()) {
      case RiskLevel::kRed: red_count++; break;
      case RiskLevel::kYellow: yellow_count++; break;
      case RiskLevel::kGreen: green_count++; break;
      default: none_count++; break;
    }
    min_distance = std::min(min_distance, cluster.GetDistance());
    max_distance = std::max(max_distance, cluster.GetDistance());
  }
  
  if (clusters.empty()) {
    min_distance = 0.0f;
  }
  
  std::ostringstream stats;
  stats << "  \"statistics\": {\n";
  stats << "    \"risk_distribution\": {\n";
  stats << "      \"red\": " << red_count << ",\n";
  stats << "      \"yellow\": " << yellow_count << ",\n";
  stats << "      \"green\": " << green_count << ",\n";
  stats << "      \"none\": " << none_count << "\n";
  stats << "    },\n";
  stats << "    \"distance_range\": {\n";
  stats << "      \"min\": " << FormatFloat(min_distance) << ",\n";
  stats << "      \"max\": " << FormatFloat(max_distance) << "\n";
  stats << "    }\n";
  stats << "  },\n";
  
  return stats.str();
}

}  // namespace brother_eye
