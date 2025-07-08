#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include "pointcloud_processor.h"
#include "json_exporter.h"
#include "dds_publisher.h"

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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      for (auto& sample : samples) {
        if (!sample.info().valid()) {
          continue;
        }
        
        processor.IncrementFrameCount();
        const auto& input_msg = sample.data();
        std::cout << "[MAIN] 📦 Frame #" << processor.GetFrameCount()
                  << " | Points = " << input_msg.data().size() / input_msg.point_step() << std::endl;

        // Process point cloud
        auto cloud_raw = processor.ConvertToPcl(input_msg);
        auto cleaned = processor.CleanCloud(cloud_raw);
        auto cluster_data = processor.ExtractClustersWithBoundingBoxes(cleaned);
        
        // Extract centroids for tracking
        std::vector<cv::Point2f> centroids;
        for (const auto& data : cluster_data) {
          centroids.push_back(data.GetCentroid());
        }
        
        // Update tracking
        processor.UpdateTracking(centroids);
        processor.CleanupOldTrackers();
        
        // Create tracked point cloud
        auto tracked_cloud = processor.CreateTrackedPointCloud();
        
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
        
        processor.IncrementFrameCount();
        
        std::cout << "[MAIN] ✅ Frame processing complete\n";
      }
    }
    
  } catch (const std::exception& e) {
    std::cerr << "[MAIN] ❌ Exception: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "[MAIN] ❌ Unknown exception occurred\n";
    return 1;
  }

  std::cout << "[MAIN] 👋 Shutdown complete.\n";
  return 0;
}
