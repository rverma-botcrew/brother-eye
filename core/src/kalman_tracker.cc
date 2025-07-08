#include "kalman_tracker.h"

namespace brother_eye {

KalmanTracker::KalmanTracker() {
  kf_ = cv::KalmanFilter(4, 2);
  state_ = cv::Mat::zeros(4, 1, CV_32F);
  measurement_ = cv::Mat::zeros(2, 1, CV_32F);

  kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
                         1, 0, 1, 0,
                         0, 1, 0, 1,
                         0, 0, 1, 0,
                         0, 0, 0, 1);
  kf_.measurementMatrix = cv::Mat::eye(2, 4, CV_32F);
  setIdentity(kf_.processNoiseCov, cv::Scalar(1e-1));     // smoother prediction
  setIdentity(kf_.measurementNoiseCov, cv::Scalar(5e-2));  // trust measurements less
  setIdentity(kf_.errorCovPost, cv::Scalar(1));
}

cv::Point2f KalmanTracker::Update(const cv::Point2f& point) {
  measurement_.at<float>(0) = point.x;
  measurement_.at<float>(1) = point.y;

  kf_.correct(measurement_);
  cv::Mat prediction = kf_.predict();

  return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

cv::Point2f KalmanTracker::Predict() {
  cv::Mat prediction = kf_.predict();
  return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

TrackedObject::TrackedObject(const cv::Point2f& initial_position)
    : kalman_filter_(), last_centroid_(initial_position), age_(0), lost_frames_(0), active_(true) {
}

void TrackedObject::Update(const cv::Point2f& new_position) {
  last_centroid_ = kalman_filter_.Update(new_position);
  lost_frames_ = 0;
  ++age_;
}

void TrackedObject::Predict() {
  last_centroid_ = kalman_filter_.Predict();
  ++lost_frames_;
}

float TrackedObject::DistanceTo(const cv::Point2f& point) const {
  return cv::norm(last_centroid_ - point);
}

}  // namespace brother_eye
