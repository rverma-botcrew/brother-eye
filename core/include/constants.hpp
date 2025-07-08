#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

#include <cmath>
#include <opencv2/opencv.hpp>

namespace brother_eye {

// =============================================================================
// POINT CLOUD FILTERING CONSTANTS
// =============================================================================
namespace FilteringConstants {
  // Voxel filtering
  constexpr float kDefaultVoxelLeafSize = 0.05f;     ///< Default voxel grid leaf size (5cm)
  constexpr float kLargeVoxelLeafSize = 0.2f;        ///< Large voxel grid leaf size (20cm) for aggressive filtering
  
  // Range filtering
  constexpr float kDefaultRangeMin = -2.0f;          ///< Default minimum Z range (meters)
  constexpr float kDefaultRangeMax = 10.0f;          ///< Default maximum Z range (meters)
  constexpr float kWideRangeMin = -10.0f;            ///< Wide minimum Z range (meters)
  constexpr float kWideRangeMax = 20.0f;             ///< Wide maximum Z range (meters)
  
  // Ground plane removal
  constexpr float kDefaultGroundThreshold = 0.02f;   ///< Default RANSAC distance threshold (2cm)
  constexpr float kRelaxedGroundThreshold = 0.2f;    ///< Relaxed RANSAC distance threshold (20cm)
  constexpr int kDefaultGroundIterations = 1000;     ///< Default maximum RANSAC iterations
  constexpr int kRelaxedGroundIterations = 30;       ///< Relaxed maximum RANSAC iterations
  
  // Statistical outlier removal
  constexpr int kDefaultOutlierMeanK = 50;           ///< Default k-neighbors for outlier removal
  constexpr int kRelaxedOutlierMeanK = 5;            ///< Relaxed k-neighbors for outlier removal
  constexpr float kDefaultOutlierStdDev = 1.0f;      ///< Default standard deviation multiplier
  constexpr float kRelaxedOutlierStdDev = 5.0f;      ///< Relaxed standard deviation multiplier
  
  // Point count thresholds
  constexpr int kMinPointsForFiltering = 100;        ///< Minimum points required for full filtering
  constexpr int kMinPointsForVoxelOnly = 50;         ///< Minimum points for voxel-only filtering
  constexpr int kMinPointsForGroundRemoval = 20;     ///< Minimum points for ground removal
  constexpr int kMinPointsForRobustProcessing = 3;   ///< Minimum points for any processing
  
  // Adaptive thresholds
  constexpr float kMinGroundRetentionRatio = 0.1f;   ///< Minimum ratio of points to retain after ground removal
  constexpr float kMinOutlierRetentionRatio = 0.3f;  ///< Minimum ratio of points to retain after outlier removal
  constexpr int kOutlierMeanKMultiplier = 2;         ///< Multiplier for outlier mean_k relative to point count
}

// =============================================================================
// CLUSTERING CONSTANTS
// =============================================================================
namespace ClusteringConstants {
  // Euclidean clustering
  constexpr float kDefaultTolerance = 0.2f;          ///< Default cluster tolerance
  constexpr int kDefaultMinClusterSize = 10;         ///< Default minimum cluster size (points)
  constexpr int kDefaultMaxClusterSize = 10000;       ///< Default maximum cluster size (points)
  
  // Bounding box
  constexpr float kMinBoundingBoxDimension = 0.1f;   ///< Minimum bounding box dimension (10cm)
  
  // Precision settings
  constexpr int kDistancePrecision = 2;              ///< Decimal places for distance display
  constexpr int kAnglePrecision = 1;                 ///< Decimal places for angle display
  constexpr int kSizePrecision = 2;                  ///< Decimal places for size display
  
  // Angle conversion
  constexpr float kRadiansToDegrees = 180.0f / M_PI; ///< Conversion factor from radians to degrees
  constexpr float kCenterCalculationDivisor = 2.0f;  ///< Divisor for calculating center coordinates
}

// =============================================================================
// CLUSTER INFO CONSTANTS (from cluster_info.hpp)
// =============================================================================
namespace ClusterInfoConstants {
  // Risk distance thresholds
  constexpr float kRedRiskDistance = 1.5f;           ///< Distance threshold for red (high) risk
  constexpr float kYellowRiskDistance = 2.0f;        ///< Distance threshold for yellow (medium) risk
  constexpr float kGreenRiskDistance = 5.0f;         ///< Distance threshold for green (low) risk
  
  // Default bounding box dimensions
  constexpr float kDefaultHeight = 0.5f;             ///< Default height center (meters)
  constexpr float kDefaultWidth = 0.5f;              ///< Default width (meters)
  constexpr float kDefaultDepth = 0.5f;              ///< Default depth (meters)
  constexpr float kDefaultTotalHeight = 1.0f;        ///< Default total height (meters)
  
  // Risk level enumeration values
  constexpr int kNoneRiskValue = 0;                  ///< No risk detected
  constexpr int kGreenRiskValue = 1;                 ///< Low risk - safe distance
  constexpr int kYellowRiskValue = 2;                ///< Medium risk - caution required
  constexpr int kRedRiskValue = 3;                   ///< High risk - immediate attention required
}

// =============================================================================
// KALMAN FILTER CONSTANTS (from kalman_tracker.hpp)
// =============================================================================
namespace KalmanConfig {
  constexpr int kStateSize = 4;                      ///< State vector size [x, y, vx, vy]
  constexpr int kMeasurementSize = 2;                ///< Measurement vector size [x, y]
  constexpr float kProcessNoise = 1e-1f;             ///< Process noise covariance
  constexpr float kMeasurementNoise = 5e-2f;         ///< Measurement noise covariance
  constexpr float kErrorCovariance = 1.0f;           ///< Initial error covariance
}

// =============================================================================
// TRACKING CONSTANTS
// =============================================================================
namespace TrackingConstants {
  // Object tracking
  constexpr float kBaseMatchingThreshold = 0.4f;     ///< Base distance threshold for object matching
  constexpr float kAdaptiveThresholdFactor = 0.05f;  ///< Adaptive factor for threshold adjustment
  constexpr float kMinMatchingThreshold = 0.4f;      ///< Minimum matching threshold
  constexpr int kInitialObjectId = 0;                ///< Initial object ID counter
  constexpr int kInvalidTrackerId = -1;              ///< Invalid tracker ID indicator
  
  // Tracking configuration (from kalman_tracker.hpp)
  constexpr int kMaxLostFrames = 5;                  ///< Maximum frames before object is considered lost
  constexpr float kMaxTrackingDistance = 2.0f;      ///< Maximum distance for object association
  
  // Tracked object visualization
  constexpr float kTrackedPointZ = 0.0f;             ///< Z coordinate for tracked object markers
  constexpr float kTrackedPointBaseIntensity = 200.0f; ///< Base intensity for tracked points
  constexpr int kTrackedPointIntensityRange = 55;    ///< Range for unique intensity per tracked object
  constexpr int kTrackedCloudHeight = 1;             ///< Height for tracked object point cloud
  
  // Kalman filter dimensions
  constexpr int kKalmanStateSize = 4;                ///< Kalman filter state size [x, y, vx, vy]
  constexpr int kKalmanMeasurementSize = 2;          ///< Kalman filter measurement size [x, y]
  
  // Kalman filter matrix values
  constexpr float kKalmanTransitionValue = 1.0f;     ///< Transition matrix diagonal values
  constexpr float kKalmanVelocityValue = 1.0f;       ///< Velocity transition values
  constexpr float kKalmanZeroValue = 0.0f;           ///< Zero values in transition matrix
  
  // Tracking lifecycle
  constexpr int kInitialAge = 0;                     ///< Initial age for new tracked objects
  constexpr int kInitialLostFrames = 0;              ///< Initial lost frames counter
  constexpr int kResetLostFrames = 0;                ///< Reset value for lost frames counter
}

// =============================================================================
// DDS PUBLISHER CONSTANTS (from dds_publisher.hpp)
// =============================================================================
namespace DdsPublisherConstants {
  // DDS configuration
  constexpr size_t kMaxClusters = 512;               ///< Maximum number of clusters
  constexpr int kHistoryDepth = 10;                  ///< History depth for DDS
  
  // Validation and matching
  constexpr float kClusterMatchDistance = 0.3f;      ///< Distance threshold for cluster matching
  
  // Risk distance thresholds (duplicated from ClusterInfoConstants for compatibility)
  constexpr float kRedRiskDistance = 1.5f;           ///< Distance threshold for red (high) risk
  constexpr float kYellowRiskDistance = 2.0f;        ///< Distance threshold for yellow (medium) risk
  constexpr float kGreenRiskDistance = 5.0f;         ///< Distance threshold for green (low) risk
  
  // Coordinate limits
  constexpr float kMaxCoordinate = 100.0f;           ///< Maximum coordinate value
  constexpr float kMinCoordinate = -100.0f;          ///< Minimum coordinate value
  constexpr float kMaxDistance = 200.0f;             ///< Maximum distance value
  constexpr float kMaxZCoordinate = 10.0f;           ///< Maximum Z coordinate
  constexpr float kMinZCoordinate = -10.0f;          ///< Minimum Z coordinate
  
  // Default values
  constexpr float kDefaultBboxSize = 0.5f;           ///< Default bounding box size
  constexpr float kDefaultZHeight = 0.5f;            ///< Default Z height
  
  // Bounding box size limits
  constexpr float kMinBboxSize = 0.01f;              ///< Minimum bounding box size
  constexpr float kMaxBboxSize = 50.0f;              ///< Maximum bounding box size
  constexpr float kMinBboxSizeForArray = 0.1f;       ///< Minimum bounding box size for array
}

// =============================================================================
// DATA CONVERSION CONSTANTS
// =============================================================================
namespace DataConversionConstants {
  // Point cloud structure
  constexpr int kPointCloudHeight = 1;               ///< Standard height for unorganized point clouds
  constexpr int kEmptyCloudWidth = 0;                ///< Width for empty point clouds
  constexpr int kEmptyCloudHeight = 0;               ///< Height for empty point clouds
  constexpr int kZeroPointStep = 0;                  ///< Zero point step (invalid)
  constexpr int kZeroPointCount = 0;                 ///< Zero point count
  
  // Point field indices
  constexpr int kXFieldIndex = 0;                    ///< X coordinate field index
  constexpr int kYFieldIndex = 1;                    ///< Y coordinate field index
  constexpr int kZFieldIndex = 2;                    ///< Z coordinate field index
  constexpr int kIntensityFieldIndex = 3;            ///< Intensity field index
}

// =============================================================================
// VISUALIZATION CONSTANTS
// =============================================================================
namespace VisualizationConstants {
  // Display image
  constexpr int kDisplayRadius = 200;                ///< Display image radius (pixels)
  constexpr int kDisplayPadding = 50;                ///< Display image padding (pixels)
  constexpr int kDisplayImageChannels = 3;           ///< Number of channels in display image (RGB)
  constexpr int kDisplayImageType = CV_8UC(3);      ///< OpenCV image type for display
  
  // Colors (BGR format for OpenCV)
  constexpr int kBackgroundColorB = 30;              ///< Background color - Blue channel
  constexpr int kBackgroundColorG = 30;              ///< Background color - Green channel
  constexpr int kBackgroundColorR = 30;              ///< Background color - Red channel
  constexpr int kCenterMarkerColorB = 255;           ///< Center marker color - Blue channel
  constexpr int kCenterMarkerColorG = 255;           ///< Center marker color - Green channel
  constexpr int kCenterMarkerColorR = 255;           ///< Center marker color - Red channel
  
  // Drawing parameters
  constexpr int kCenterMarkerRadius = 3;             ///< Center marker circle radius
  constexpr int kCenterMarkerThickness = -1;         ///< Center marker thickness (filled)
  constexpr int kTextPositionX = 10;                 ///< Text position X coordinate
  constexpr int kTextPositionY = 20;                 ///< Text position Y coordinate
  constexpr double kTextFontScale = 0.6;             ///< Text font scale
  constexpr int kTextThickness = 1;                  ///< Text thickness
}

// =============================================================================
// JSON EXPORT CONSTANTS (from json_exporter.hpp)
// =============================================================================
namespace JsonConstants {
  // Formatting
  constexpr int kDefaultFloatPrecision = 3;          ///< Default decimal precision for float values
  constexpr float kClusterMatchThreshold = 0.3f;     ///< Distance threshold for matching clusters
  constexpr const char* kDefaultOutputFile = "/tmp/clusters.json";  ///< Default output file path
  constexpr int kInvalidTrackingId = -1;             ///< Invalid tracking ID
  constexpr int kInitialAge = 0;                     ///< Initial age for new objects
  constexpr int kInitialLostFrames = 0;              ///< Initial lost frames counter
  
  // Statistics initialization
  constexpr int kZeroRiskCount = 0;                  ///< Zero risk count
  constexpr float kZeroDistance = 0.0f;              ///< Zero distance value
  constexpr int kZeroRiskLevel = 0;                  ///< Zero risk level
  
  // Risk level mapping
  constexpr int kRedRiskLevel = 3;                   ///< Red risk level value
  constexpr int kYellowRiskLevel = 2;                ///< Yellow risk level value
  constexpr int kGreenRiskLevel = 1;                 ///< Green risk level value
  constexpr int kNoneRiskLevel = 0;                  ///< No risk level value
  
  // Timestamp formatting
  constexpr int kMillisecondsPerSecond = 1000;       ///< Milliseconds per second
  constexpr int kTimestampWidth = 3;                 ///< Width for timestamp formatting
  constexpr char kTimestampFillChar = '0';           ///< Fill character for timestamp
}

// =============================================================================
// DDS CONSTANTS
// =============================================================================
namespace DdsConstants {
  // Domain and topic
  constexpr int kDefaultDomainId = 0;                ///< Default DDS domain ID
  constexpr int kEmptyClusterSize = 0;               ///< Empty cluster array size
  
  // Message validation
  constexpr int kZeroAge = 0;                        ///< Zero age (invalid)
  constexpr int kZeroLostFrames = 0;                 ///< Zero lost frames
  
  // Type casting
  constexpr int kInt32Cast = 32;                     ///< Bit size for int32 cast
  constexpr int kUInt32Cast = 32;                    ///< Bit size for uint32 cast
}

// =============================================================================
// SYSTEM CONSTANTS
// =============================================================================
namespace SystemConstants {
  // Main loop
  constexpr int kMainLoopDelayMs = 50;               ///< Main loop delay in milliseconds
  constexpr int kSuccessExitCode = 0;                ///< Success exit code
  constexpr int kErrorExitCode = 1;                  ///< Error exit code
  
  // Frame counting
  constexpr int kInitialFrameCount = 0;              ///< Initial frame count
}

}  // namespace brother_eye
#endif  // CONSTANTS_HPP_
