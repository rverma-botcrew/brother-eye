#include "kalman_tracker.hpp"
#include "constants.hpp"
#include <iostream>

namespace brother_eye {

KalmanTracker::KalmanTracker() 
    : kalman_filter_(KalmanConfig::kStateSize, KalmanConfig::kMeasurementSize),
      state_(cv::Mat::zeros(KalmanConfig::kStateSize, 1, CV_32F)),
      measurement_(cv::Mat::zeros(KalmanConfig::kMeasurementSize, 1, CV_32F)) {
  
  InitializeMatrices();
}

void KalmanTracker::InitializeMatrices() {
  // Transition matrix for constant velocity model
  // State: [x, y, vx, vy] -> [x+vx, y+vy, vx, vy]
  kalman_filter_.transitionMatrix = (cv::Mat_<float>(TrackingConstants::kKalmanStateSize, TrackingConstants::kKalmanStateSize) <<
    TrackingConstants::kKalmanTransitionValue, TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanVelocityValue, TrackingConstants::kKalmanZeroValue,   // x_new = x + vx
    TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanTransitionValue, TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanVelocityValue,   // y_new = y + vy
    TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanTransitionValue, TrackingConstants::kKalmanZeroValue,   // vx_new = vx
    TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanZeroValue, TrackingConstants::kKalmanTransitionValue    // vy_new = vy
  );
  
  // Measurement matrix - we only observe position [x, y]
  kalman_filter_.measurementMatrix = cv::Mat::eye(KalmanConfig::kMeasurementSize, 
                                                  KalmanConfig::kStateSize, CV_32F);
  
  // Process noise covariance - model uncertainty
  setIdentity(kalman_filter_.processNoiseCov, cv::Scalar(KalmanConfig::kProcessNoise));
  
  // Measurement noise covariance - sensor uncertainty
  setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar(KalmanConfig::kMeasurementNoise));
  
  // Initial error covariance
  setIdentity(kalman_filter_.errorCovPost, cv::Scalar(KalmanConfig::kErrorCovariance));
}

cv::Point2f KalmanTracker::Update(const cv::Point2f& point) {
  // Set measurement values
  measurement_.at<float>(DataConversionConstants::kXFieldIndex) = point.x;
  measurement_.at<float>(DataConversionConstants::kYFieldIndex) = point.y;
  
  // Correct the state with the measurement
  kalman_filter_.correct(measurement_);
  
  // Predict the next state
  const cv::Mat prediction = kalman_filter_.predict();
  
  return cv::Point2f(prediction.at<float>(DataConversionConstants::kXFieldIndex), prediction.at<float>(DataConversionConstants::kYFieldIndex));
}

cv::Point2f KalmanTracker::Predict() {
  const cv::Mat prediction = kalman_filter_.predict();
  return cv::Point2f(prediction.at<float>(DataConversionConstants::kXFieldIndex), prediction.at<float>(DataConversionConstants::kYFieldIndex));
}

TrackedObject::TrackedObject(const cv::Point2f& initial_position)
    : kalman_filter_(), 
      last_centroid_(initial_position), 
      predicted_position_(initial_position),
      age_(TrackingConstants::kInitialAge), 
      lost_frames_(TrackingConstants::kInitialLostFrames), 
      active_(true) {
  
  // Initialize the Kalman filter with the first measurement
  kalman_filter_.Update(initial_position);
}

void TrackedObject::Update(const cv::Point2f& new_position) {
  // Update the filter with the new measurement
  predicted_position_ = kalman_filter_.Update(new_position);
  last_centroid_ = new_position;
  
  // Reset lost frames counter and increment age
  lost_frames_ = TrackingConstants::kResetLostFrames;
  ++age_;
}

void TrackedObject::Predict() {
  // Predict the next position without measurement
  predicted_position_ = kalman_filter_.Predict();
  last_centroid_ = predicted_position_;
  
  // Increment lost frames counter
  ++lost_frames_;
  
  // Deactivate object if lost for too long
  if (lost_frames_ >= TrackingConstants::kMaxLostFrames) {
    active_ = false;
  }
}

float TrackedObject::DistanceTo(const cv::Point2f& point) const {
  return cv::norm(last_centroid_ - point);
}

bool TrackedObject::IsLost() const {
  return lost_frames_ >= TrackingConstants::kMaxLostFrames;
}

}  // namespace brother_eye
