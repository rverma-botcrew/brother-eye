#ifndef KALMAN_TRACKER_HPP_
#define KALMAN_TRACKER_HPP_

#include <opencv2/opencv.hpp>
#include "constants.hpp"

namespace brother_eye {

/**
 * @brief Kalman filter implementation for 2D position tracking
 * 
 * This class implements a Kalman filter for tracking 2D positions with velocity.
 * The state vector contains [x, y, vx, vy] and measurements are [x, y].
 */
class KalmanTracker {
 public:
  /**
   * @brief Constructs a Kalman tracker with default configuration
   */
  KalmanTracker();
  
  /**
   * @brief Updates the filter with a new measurement and returns predicted position
   * @param point The measured position
   * @return The predicted position after correction
   */
  cv::Point2f Update(const cv::Point2f& point);
  
  /**
   * @brief Predicts the next position without measurement
   * @return The predicted position
   */
  cv::Point2f Predict();
  
 private:
  cv::KalmanFilter kalman_filter_;     ///< OpenCV Kalman filter instance
  cv::Mat state_;                      ///< State vector [x, y, vx, vy]
  cv::Mat measurement_;                ///< Measurement vector [x, y]
  
  /**
   * @brief Initializes the Kalman filter matrices
   */
  void InitializeMatrices();
};

/**
 * @brief Tracks objects over time using Kalman filtering
 * 
 * This class represents a tracked object with position history, age tracking,
 * and loss detection capabilities. It uses a Kalman filter for position prediction
 * and maintains tracking state information.
 */
class TrackedObject {
 public:
  /**
   * @brief Default constructor for empty tracked object
   */
  TrackedObject() = default;
  
  /**
   * @brief Constructs a tracked object with initial position
   * @param initial_position The starting position of the object
   */
  explicit TrackedObject(const cv::Point2f& initial_position);
  
  /**
   * @brief Updates the object with a new observed position
   * @param new_position The newly observed position
   */
  void Update(const cv::Point2f& new_position);
  
  /**
   * @brief Predicts the next position when no measurement is available
   * This increments the lost frames counter
   */
  void Predict();
  
  /**
   * @brief Calculates Euclidean distance to a given point
   * @param point The point to calculate distance to
   * @return Distance in the same units as the coordinate system
   */
  float DistanceTo(const cv::Point2f& point) const;
  
  /**
   * @brief Checks if the object should be considered lost
   * @return true if the object has been lost for too many frames
   */
  bool IsLost() const;
  
  // Getters
  cv::Point2f GetLastCentroid() const noexcept { return last_centroid_; }
  cv::Point2f GetPredictedPosition() const noexcept { return predicted_position_; }
  int GetAge() const noexcept { return age_; }
  int GetLostFrames() const noexcept { return lost_frames_; }
  bool IsActive() const noexcept { return active_; }
  
  // Setters
  void SetActive(bool active) { active_ = active; }
  
 private:
  KalmanTracker kalman_filter_;              ///< Kalman filter for position tracking
  cv::Point2f last_centroid_;                ///< Last known/predicted position
  cv::Point2f predicted_position_;           ///< Predicted position for next frame
  int age_ = 0;                              ///< Number of frames this object has been tracked
  int lost_frames_ = 0;                      ///< Number of consecutive frames without detection
  bool active_ = true;                       ///< Whether the object is actively being tracked
};

}  // namespace brother_eye

#endif  // KALMAN_TRACKER_HPP_
