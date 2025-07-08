#ifndef JSON_EXPORTER_H_
#define JSON_EXPORTER_H_

#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

#include "cluster_info.h"
#include "kalman_tracker.h"

namespace brother_eye {

class JsonExporter {
 public:
  JsonExporter() = default;
  ~JsonExporter() = default;
  
  // Main export functions
  std::string ClustersToJson(const std::vector<ClusterInfo>& clusters, 
                           const std::map<int, TrackedObject>& tracked_objects,
                           size_t frame_number);
  
  void PublishJsonToFile(const std::string& json_data, 
                        const std::string& filename = "/tmp/clusters.json");
  
  void PublishJsonToConsole(const std::string& json_data);
  
 private:
  // Helper functions
  std::string EscapeJsonString(const std::string& input);
  std::string FormatFloat(float value, int precision = 3);
  std::string ClusterInfoToJson(const ClusterInfo& cluster, 
                               int tracked_id = -1, 
                               int age = 0, 
                               int lost_frames = 0);
  
  std::string GenerateTimestamp();
  std::string GenerateStatistics(const std::vector<ClusterInfo>& clusters);
};

}  // namespace brother_eye

#endif  // JSON_EXPORTER_H_
