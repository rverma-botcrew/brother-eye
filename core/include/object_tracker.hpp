#ifndef OBJECT_TRACKER_HPP_
#define OBJECT_TRACKER_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <memory>

#include "kalman_tracker.hpp"
#include "cluster_info.hpp"

namespace brother_eye {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

/**
 * @brief Object tracking manager
 * 
 * Manages multiple tracked objects using Kalman filters for prediction
 * and nearest neighbor matching for data association.
 */
class ObjectTracker {
 public:
  /**
   * @brief Constructs a new ObjectTracker
   */
  ObjectTracker();
  
  /**
   * @brief Updates tracking with new centroid positions
   * @param centroids Vector of cluster centroids
   */
  void UpdateTracking(const std::vector<cv::Point2f>& centroids);
  
  /**
   * @brief Removes old trackers that have been lost too long
   * @param max_lost_frames Maximum frames before tracker is removed
   */
  void CleanupOldTrackers(int max_lost_frames = TrackingConstants::kMaxLostFrames);
  
  /**
   * @brief Creates a point cloud representing tracked objects
   * @return Point cloud with tracked object positions
   */
  CloudT::Ptr CreateTrackedPointCloud();
  
  /**
   * @brief Gets the currently tracked objects
   * @return Const reference to tracked objects map
   */
  const std::map<int, TrackedObject>& GetTrackedObjects() const noexcept {
    return tracked_objects_;
  }
  
  /**
   * @brief Gets the number of currently tracked objects
   * @return Number of tracked objects
   */
  size_t GetTrackedObjectCount() const noexcept {
    return tracked_objects_.size();
  }
  
 private:
  std::map<int, TrackedObject> tracked_objects_;  ///< Currently tracked objects
  int next_id_;                                   ///< Next available object ID
  
  /**
   * @brief Calculates adaptive threshold for object matching
   * @return Adaptive threshold value based on current tracker count
   */
  float CalculateAdaptiveThreshold() const;
};

}  // namespace brother_eye

#endif  // OBJECT_TRACKER_HPP_
