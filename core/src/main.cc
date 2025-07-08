#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include "pointcloud_processor.hpp"
#include "json_exporter.hpp"
#include "dds_publisher.hpp"
#include "constants.hpp"

namespace {
static bool g_running = true;
void SignalHandler(int) { 
  g_running = false; 
  std::cout << "\n[MAIN] 🛑 Received shutdown signal\n";
}
}  // namespace

int main() {
  std::signal(SIGINT, SignalHandler);
  std::cout << "[MAIN] 🚀 Initializing PointCloud filter node...\n";

  try {
    // Initialize components
    brother_eye::PointCloudProcessor processor;
    brother_eye::JsonExporter json_exporter;
    brother_eye::DdsPublisher dds_publisher;
    
    // Initialize DDS
    dds_publisher.Initialize();
    
    std::cout << "[MAIN] ✅ All components initialized successfully\n";
    
    // Main processing loop
    while (g_running) {
      auto samples = dds_publisher.GetReader()->take();
      if (samples.length() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(brother_eye::SystemConstants::kMainLoopDelayMs));
        continue;
      }

      for (auto& sample : samples) {
        if (!sample.info().valid()) {
          continue;
        }
        
        const auto& input_msg = sample.data();
        std::cout << "[MAIN] 📦 Frame #" << processor.GetFrameCount()
                  << " | Points = " << input_msg.data().size() / input_msg.point_step() << std::endl;

        // Process point cloud using refactored method
        auto tracked_cloud = processor.ProcessPointCloud(input_msg);
        
        // Get cluster data and analysis using separate methods
        auto cloud_raw = processor.ConvertToPcl(input_msg);
        auto cleaned = processor.CleanCloud(cloud_raw);
        auto cluster_data = processor.ExtractClustersWithBoundingBoxes(cleaned);
        
        // Extract centroids for tracking (this is now done internally but we need it for analysis)
        std::vector<cv::Point2f> centroids;
        for (const auto& data : cluster_data) {
          centroids.push_back(data.GetCentroid());
        }
        // Analyze cluster risk
        auto cluster_analysis = processor.AnalyzeClusterRisk(cluster_data);
        processor.DisplayClusterRiskUi(cluster_analysis);
        
        // Export JSON
        std::string json_data = json_exporter.ClustersToJson(
            cluster_analysis, 
            processor.GetTrackedObjects(), 
            processor.GetFrameCount()
        );
        json_exporter.PublishJsonToFile(json_data, "/tmp/clusters.json");
        
        // Publish via DDS
        dds_publisher.PublishPointCloud(tracked_cloud, input_msg);
        dds_publisher.PublishClusterAnalysis(cluster_analysis, processor.GetTrackedObjects());
        dds_publisher.PublishBoundingBoxes(cluster_analysis);
        
        std::cout << "[MAIN] ✅ Frame processing complete\n";
      }
    }
    
  } catch (const std::exception& e) {
    std::cerr << "[MAIN] ❌ Exception: " << e.what() << std::endl;
    return brother_eye::SystemConstants::kErrorExitCode;
  } catch (...) {
    std::cerr << "[MAIN] ❌ Unknown exception occurred\n";
    return brother_eye::SystemConstants::kErrorExitCode;
  }

  std::cout << "[MAIN] 👋 Shutdown complete.\n";
  return brother_eye::SystemConstants::kSuccessExitCode;
}
