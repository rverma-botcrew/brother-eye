#ifndef KALMAN_TRACKER_H_
#define KALMAN_TRACKER_H_

#include <opencv2/opencv.hpp>

namespace brother_eye {

class KalmanTracker {
 public:
  KalmanTracker();
  
  cv::Point2f Update(const cv::Point2f& point);
  cv::Point2f Predict();
  
 private:
  cv::KalmanFilter kf_;
  cv::Mat state_;      // [x, y, vx, vy]
  cv::Mat measurement_;       // [x, y]
};

class TrackedObject {
 public:
  TrackedObject() = default;
  explicit TrackedObject(const cv::Point2f& initial_position);
  
  void Update(const cv::Point2f& new_position);
  void Predict();
  float DistanceTo(const cv::Point2f& point) const;
  
  // Getters
  cv::Point2f GetLastCentroid() const { return last_centroid_; }
  int GetAge() const { return age_; }
  int GetLostFrames() const { return lost_frames_; }
  bool IsActive() const { return active_; }
  
 private:
  KalmanTracker kalman_filter_;
  cv::Point2f last_centroid_;
  int age_ = 0;
  int lost_frames_ = 0;
  bool active_ = true;
};

}  // namespace brother_eye

#endif  // KALMAN_TRACKER_H_
