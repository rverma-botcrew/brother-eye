#ifndef JSON_EXPORTER_H_
#define JSON_EXPORTER_H_

#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

#include "cluster_info.h"
#include "kalman_tracker.h"

namespace brother_eye {

// Constants for JSON export configuration
namespace JsonExportConfig {
  constexpr int kDefaultFloatPrecision = 3;      ///< Default decimal precision for float values
  constexpr float kClusterMatchThreshold = 0.3f;  ///< Distance threshold for matching clusters to tracked objects
  constexpr const char* kDefaultOutputFile = "/tmp/clusters.json";  ///< Default output file path
}

/**
 * @brief Handles exporting cluster information to JSON format
 * 
 * This class provides functionality to convert cluster data and tracking information
 * into structured JSON format for analysis, visualization, or data storage.
 */
class JsonExporter {
 public:
  JsonExporter() = default;
  ~JsonExporter() = default;
  
  // Delete copy constructor and assignment operator to prevent copying
  JsonExporter(const JsonExporter&) = delete;
  JsonExporter& operator=(const JsonExporter&) = delete;
  
  /**
   * @brief Converts cluster data to JSON format
   * @param clusters Vector of detected cluster information
   * @param tracked_objects Map of tracked objects with their IDs
   * @param frame_number Current frame number for tracking
   * @return JSON string representation of the cluster data
   */
  std::string ClustersToJson(const std::vector<ClusterInfo>& clusters, 
                           const std::map<int, TrackedObject>& tracked_objects,
                           size_t frame_number);
  
  /**
   * @brief Publishes JSON data to a file
   * @param json_data The JSON string to write
   * @param filename Output file path (default: /tmp/clusters.json)
   */
  void PublishJsonToFile(const std::string& json_data, 
                        const std::string& filename = JsonExportConfig::kDefaultOutputFile);
  
  /**
   * @brief Publishes JSON data to console output
   * @param json_data The JSON string to display
   */
  void PublishJsonToConsole(const std::string& json_data);
  
 private:
  /**
   * @brief Escapes special characters in strings for JSON compatibility
   * @param input The input string to escape
   * @return Escaped string safe for JSON
   */
  std::string EscapeJsonString(const std::string& input);
  
  /**
   * @brief Formats float values to specified precision
   * @param value The float value to format
   * @param precision Number of decimal places (default: 3)
   * @return Formatted float as string
   */
  std::string FormatFloat(float value, int precision = JsonExportConfig::kDefaultFloatPrecision);
  
  /**
   * @brief Converts a single cluster to JSON format
   * @param cluster The cluster information to convert
   * @param tracked_id Associated tracking ID (-1 if not tracked)
   * @param age Age of the tracked object in frames
   * @param lost_frames Number of frames since last detection
   * @return JSON string representation of the cluster
   */
  std::string ClusterInfoToJson(const ClusterInfo& cluster, 
                               int tracked_id = -1, 
                               int age = 0, 
                               int lost_frames = 0);
  
  /**
   * @brief Generates timestamp information for JSON output
   * @return JSON string with timestamp data
   */
  std::string GenerateTimestamp();
  
  /**
   * @brief Generates statistical summary of cluster data
   * @param clusters Vector of cluster information for analysis
   * @return JSON string with statistical information
   */
  std::string GenerateStatistics(const std::vector<ClusterInfo>& clusters);

  /**
   * @brief Generates JSON representation of cluster risk information
   * @param cluster The cluster to extract risk information from
   * @return JSON string with risk level and name
   */
  std::string GenerateRiskJson(const ClusterInfo& cluster);

  /**
   * @brief Generates JSON representation of cluster bounding box
   * @param cluster The cluster to extract bounding box from
   * @return JSON string with bounding box center, size, and volume
   */
  std::string GenerateBoundingBoxJson(const ClusterInfo& cluster);

  /**
   * @brief Generates empty statistics JSON when no clusters are present
   * @return JSON string with zero statistics
   */
  std::string GenerateEmptyStatistics();
};

}  // namespace brother_eye

#endif  // JSON_EXPORTER_H_
