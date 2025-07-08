#include "object_tracker.hpp"
#include "constants.hpp"

#include <iostream>
#include <algorithm>
#include <set>

namespace brother_eye {

ObjectTracker::ObjectTracker() : next_id_(TrackingConstants::kInitialObjectId) {
}

void ObjectTracker::UpdateTracking(const std::vector<cv::Point2f>& centroids) {
  // Predict positions for all existing trackers
  for (auto& [id, tracker] : tracked_objects_) {
    tracker.Predict();
  }

  // Perform nearest neighbor matching with adaptive threshold
  std::set<int> matched_tracker_ids;
  
  for (const auto& centroid : centroids) {
    int best_tracker_id = TrackingConstants::kInvalidTrackerId;
    float best_distance = CalculateAdaptiveThreshold();
    
    // Find the closest unmatched tracker
    for (auto& [tracker_id, tracker] : tracked_objects_) {
      if (matched_tracker_ids.count(tracker_id) > TrackingConstants::kInitialObjectId) continue;
      
      const float distance = tracker.DistanceTo(centroid);
      if (distance < best_distance) {
        best_distance = distance;
        best_tracker_id = tracker_id;
      }
    }

    if (best_tracker_id == TrackingConstants::kInvalidTrackerId) {
      // Create new tracker for unmatched detection
      tracked_objects_[next_id_] = TrackedObject(centroid);
      matched_tracker_ids.insert(next_id_);
      std::cout << "[TRACKER] 🆕 New object ID: " << next_id_ << std::endl;
      ++next_id_;
    } else {
      // Update existing tracker
      tracked_objects_[best_tracker_id].Update(centroid);
      matched_tracker_ids.insert(best_tracker_id);
    }
  }
}

float ObjectTracker::CalculateAdaptiveThreshold() const {
  // Adaptive threshold based on number of tracked objects
  return std::max(TrackingConstants::kBaseMatchingThreshold, 
                  TrackingConstants::kMinMatchingThreshold - TrackingConstants::kAdaptiveThresholdFactor * tracked_objects_.size());
}

void ObjectTracker::CleanupOldTrackers(int max_lost_frames) {
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if (it->second.GetLostFrames() > max_lost_frames) {
      std::cout << "[TRACKER] ❌ Removing lost object ID: " << it->first << std::endl;
      it = tracked_objects_.erase(it);
    } else {
      ++it;
    }
  }
}

CloudT::Ptr ObjectTracker::CreateTrackedPointCloud() {
  CloudT::Ptr tracked_cloud(new CloudT);
  tracked_cloud->points.reserve(tracked_objects_.size());
  
  for (const auto& [id, tracker] : tracked_objects_) {
    PointT point;
    point.x = tracker.GetLastCentroid().x;
    point.y = tracker.GetLastCentroid().y;
    point.z = TrackingConstants::kTrackedPointZ;
    point.intensity = TrackingConstants::kTrackedPointBaseIntensity + (id % TrackingConstants::kTrackedPointIntensityRange);  // Unique intensity per ID
    tracked_cloud->points.push_back(point);
  }
  
  tracked_cloud->width = tracked_cloud->points.size();
  tracked_cloud->height = TrackingConstants::kTrackedCloudHeight;
  tracked_cloud->is_dense = true;
  
  return tracked_cloud;
}

}  // namespace brother_eye
