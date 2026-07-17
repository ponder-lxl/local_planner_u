#include <cmath>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "polar_gap_planner/polar_analyzer.h"

namespace {
const float kPi = 3.1415926f;

enum PlannerState {
  STATE_ESCAPE = 0,
  STATE_NAVIGATE = 1,
  STATE_DONE = 2,
  STATE_STUCK = 3,
  STATE_CIRCUM = 4
};

std::string stateToString(int state) {
  switch (state) {
    case STATE_ESCAPE:
      return "ESCAPE";
    case STATE_NAVIGATE:
      return "NAVIGATE";
    case STATE_DONE:
      return "DONE";
    case STATE_STUCK:
      return "STUCK";
    case STATE_CIRCUM:
      return "CIRCUM";
    default:
      return "UNKNOWN";
  }
}

void vehicleToMap(float xVehicle, float yVehicle, float vehicleX, float vehicleY, float vehicleYaw,
                  float* xMap, float* yMap) {
  const float cosYaw = std::cos(vehicleYaw);
  const float sinYaw = std::sin(vehicleYaw);
  *xMap = vehicleX + xVehicle * cosYaw - yVehicle * sinYaw;
  *yMap = vehicleY + xVehicle * sinYaw + yVehicle * cosYaw;
}

float normalizeAngle(float angleRad) {
  while (angleRad > kPi) {
    angleRad -= 2.0f * kPi;
  }
  while (angleRad < -kPi) {
    angleRad += 2.0f * kPi;
  }
  return angleRad;
}
}  // namespace

class PolarGapPlannerNode {
 public:
  PolarGapPlannerNode()
      : nh_(),
        pnh_("~"),
        analyzer_(),
        hasOdom_(false),
        hasGoal_(false),
        hasTerrainCloud_(false),
        hasScanCloud_(false),
        state_(STATE_ESCAPE),
        lastLoggedState_(-1),
        verboseLog_(true),
        logThrottleSec_(1.0),
        hasExitBearing_(false),
        exitBearingVehicle_(0.0f),
        lastEscapeBearing_(0.0f),
        lastEscapeGapValid_(false),
        circumSideSign_(0),
        circumBlockedTicks_(0) {
    loadParameters();
    configureAnalyzer();

    if (verboseLog_) {
      ROS_WARN("[polar_gap] init | terrain=%d(%s) scan=%d(%s) | min_free=%.2f gap_free=%.2f min_gap=%.1fdeg "
               "nav_free=%.2f esc_free=%.2f | log_throttle=%.1fs",
               useTerrainMap_, terrainTopic_.c_str(), useRegisteredScan_, scanTopic_.c_str(),
               minFreeDistance_, gapFreeDistance_, minGapWidthDeg_, navigateFreeDist_, escapeFreeDist_,
               logThrottleSec_);
    }

    subOdom_ = nh_.subscribe("/state_estimation", 5, &PolarGapPlannerNode::odomHandler, this);
    subTerrain_ = nh_.subscribe(terrainTopic_, 5, &PolarGapPlannerNode::terrainHandler, this);
    subScan_ = nh_.subscribe(scanTopic_, 5, &PolarGapPlannerNode::scanHandler, this);
    subGoalPose_ = nh_.subscribe("/final_goal", 5, &PolarGapPlannerNode::goalPoseHandler, this);
    subGoalPoint_ = nh_.subscribe("/final_goal_point", 5, &PolarGapPlannerNode::goalPointHandler, this);
    subNavGoal_ = nh_.subscribe("/move_base_simple/goal", 5, &PolarGapPlannerNode::navGoalHandler, this);

    pubWaypoint_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 5);
    pubSpeed_ = nh_.advertise<std_msgs::Float32>("/speed", 5);
    pubState_ = nh_.advertise<std_msgs::String>("/planner_state", 1);
    pubMarkers_ = nh_.advertise<visualization_msgs::MarkerArray>("/polar_gap_markers", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / replanRate_), &PolarGapPlannerNode::timerCallback, this);

    ROS_INFO("polar_gap_planner started. Set final goal on /final_goal, /final_goal_point, or RViz 2D Nav Goal.");
  }

 private:
  void loadParameters() {
    pnh_.param("scan_range", scanRange_, 5.0);
    pnh_.param("bin_num", binNum_, 360);
    pnh_.param("obstacle_height_thre", obstacleHeightThre_, 0.15);
    pnh_.param("min_obstacle_z", minObstacleZ_, 0.15);
    pnh_.param("max_obstacle_z", maxObstacleZ_, 1.5);
    pnh_.param("min_free_distance", minFreeDistance_, 1.2);
    pnh_.param("gap_free_distance", gapFreeDistance_, 2.2);
    pnh_.param("min_gap_width_deg", minGapWidthDeg_, 30.0);
    pnh_.param("subgoal_dist", subgoalDist_, 1.5);
    pnh_.param("min_subgoal_dist", minSubgoalDist_, 1.0);
    pnh_.param("max_subgoal_dist", maxSubgoalDist_, 2.0);
    pnh_.param("goal_tolerance", goalTolerance_, 0.8);
    pnh_.param("navigate_free_dist", navigateFreeDist_, 3.0);
    pnh_.param("escape_free_dist", escapeFreeDist_, 1.5);
    pnh_.param("replan_rate", replanRate_, 10.0);
    pnh_.param("speed", speed_, 1.0);
    pnh_.param("use_terrain_map", useTerrainMap_, true);
    pnh_.param("use_registered_scan", useRegisteredScan_, true);
    pnh_.param("terrain_topic", terrainTopic_, std::string("/terrain_map"));
    pnh_.param("scan_topic", scanTopic_, std::string("/registered_scan"));
    pnh_.param("publish_markers", publishMarkers_, true);
    pnh_.param("goal_check_half_width_deg", goalCheckHalfWidthDeg_, 8.0);
    pnh_.param("navigate_cone_half_deg", navigateConeHalfDeg_, 110.0);
    pnh_.param("no_return_angle_deg", noReturnAngleDeg_, 95.0);
    pnh_.param("cavity_free_ratio", cavityFreeRatio_, 0.85);
    pnh_.param("forward_max_angle_deg", forwardMaxAngleDeg_, 75.0);
    pnh_.param("direct_navigate_max_deg", directNavigateMaxDeg_, 70.0);
    pnh_.param("skirt_angle_deg", skirtAngleDeg_, 50.0);
    pnh_.param("circum_cone_half_deg", circumConeHalfDeg_, 55.0);
    pnh_.param("circum_lateral_deg", circumLateralDeg_, 75.0);
    pnh_.param("circum_navigate_free_dist", circumNavigateFreeDist_, 1.5);
    pnh_.param("circum_escape_blocked_ticks", circumEscapeBlockedTicks_, 5);
    pnh_.param("verbose_log", verboseLog_, true);
    pnh_.param("log_throttle_sec", logThrottleSec_, 1.0);
  }

  static double radToDeg(float rad) { return static_cast<double>(rad) * 180.0 / kPi; }

  void logStateTransition(int newState, const std::string& reason) {
    if (newState == lastLoggedState_) {
      return;
    }
    ROS_WARN("[polar_gap] STATE %s -> %s | %s",
             stateToString(lastLoggedState_).c_str(), stateToString(newState).c_str(), reason.c_str());
    lastLoggedState_ = newState;
  }

  float goalPathMinRange(float minDist, float* checkDistOut) const {
    const float bearing = goalBearingVehicle();
    const float distGoal = distanceToGoal();
    const float checkDist = std::min(distGoal, minDist);
    const int halfBins = std::max(1, static_cast<int>(goalCheckHalfWidthDeg_ / (360.0 / binNum_)));
    float minRange = analyzer_.getFreeRangeAt(bearing);

    for (int offset = -halfBins; offset <= halfBins; ++offset) {
      const float angle = bearing + static_cast<float>(offset) * analyzer_.binWidthRad();
      minRange = std::min(minRange, analyzer_.getFreeRangeAt(angle));
    }
    if (checkDistOut) {
      *checkDistOut = checkDist;
    }
    return minRange;
  }

  int countBlockedBins() const {
    int blocked = 0;
    for (int i = 0; i < binNum_; ++i) {
      if (analyzer_.freeRanges()[i] < minFreeDistance_) {
        ++blocked;
      }
    }
    return blocked;
  }

  int countGapBlockedBins() const {
    int blocked = 0;
    for (int i = 0; i < binNum_; ++i) {
      if (analyzer_.freeRanges()[i] < gapFreeDistance_) {
        ++blocked;
      }
    }
    return blocked;
  }

  int countUpdatedBins() const {
    int updated = 0;
    for (int i = 0; i < binNum_; ++i) {
      if (analyzer_.freeRanges()[i] < scanRange_ - 0.01) {
        ++updated;
      }
    }
    return updated;
  }

  void configureAnalyzer() {
    analyzer_.setScanRange(static_cast<float>(scanRange_));
    analyzer_.setBinNum(binNum_);
    analyzer_.setMinFreeDistance(static_cast<float>(minFreeDistance_));
    analyzer_.setGapFreeDistance(static_cast<float>(gapFreeDistance_));
    analyzer_.setMinGapWidthDeg(static_cast<float>(minGapWidthDeg_));
    analyzer_.setObstacleHeightThre(static_cast<float>(obstacleHeightThre_));
    analyzer_.setMinObstacleZ(static_cast<float>(minObstacleZ_));
    analyzer_.setMaxObstacleZ(static_cast<float>(maxObstacleZ_));
    analyzer_.setUseTerrainMap(useTerrainMap_);
  }

  void odomHandler(const nav_msgs::Odometry::ConstPtr& msg) {
    vehicleX_ = msg->pose.pose.position.x;
    vehicleY_ = msg->pose.pose.position.y;
    vehicleZ_ = msg->pose.pose.position.z;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(vehicleRoll_, vehiclePitch_, vehicleYaw_);
    hasOdom_ = true;
  }

  void terrainHandler(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!useTerrainMap_) {
      return;
    }
    latestTerrain_ = msg;
    hasTerrainCloud_ = true;
  }

  void scanHandler(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!useRegisteredScan_) {
      return;
    }
    latestScan_ = msg;
    hasScanCloud_ = true;
  }

  void setFinalGoal(double x, double y, double z) {
    goalX_ = x;
    goalY_ = y;
    goalZ_ = z;
    hasGoal_ = true;
    state_ = STATE_ESCAPE;
    hasExitBearing_ = false;
    lastEscapeBearing_ = 0.0f;
    lastEscapeGapValid_ = false;
    circumSideSign_ = 0;
    circumBlockedTicks_ = 0;
    lastLoggedState_ = -1;
    ROS_WARN("[polar_gap] FINAL_GOAL (%.2f, %.2f, %.2f) | vehicle (%.2f, %.2f) yaw=%.1fdeg | ESCAPE",
             x, y, z, vehicleX_, vehicleY_, radToDeg(static_cast<float>(vehicleYaw_)));
  }

  void goalPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    setFinalGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void goalPointHandler(const geometry_msgs::PointStamped::ConstPtr& msg) {
    setFinalGoal(msg->point.x, msg->point.y, msg->point.z);
  }

  void navGoalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    setFinalGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  float goalBearingVehicle() const {
    const float dx = static_cast<float>(goalX_ - vehicleX_);
    const float dy = static_cast<float>(goalY_ - vehicleY_);
    const float goalYaw = std::atan2(dy, dx);
    return normalizeAngle(goalYaw - static_cast<float>(vehicleYaw_));
  }

  float distanceToGoal() const {
    const float dx = static_cast<float>(goalX_ - vehicleX_);
    const float dy = static_cast<float>(goalY_ - vehicleY_);
    return std::sqrt(dx * dx + dy * dy);
  }

  bool hasObstacleCloud() const {
    return (useTerrainMap_ && hasTerrainCloud_) || (useRegisteredScan_ && hasScanCloud_);
  }

  void updateAnalyzerClouds() {
    const float vx = static_cast<float>(vehicleX_);
    const float vy = static_cast<float>(vehicleY_);
    const float vz = static_cast<float>(vehicleZ_);
    const float vyaw = static_cast<float>(vehicleYaw_);

    analyzer_.reset();

    if (useTerrainMap_ && hasTerrainCloud_) {
      pcl::PointCloud<pcl::PointXYZI> terrainCloud;
      pcl::fromROSMsg(*latestTerrain_, terrainCloud);
      analyzer_.appendObstacles(terrainCloud, vx, vy, vz, vyaw, true);
    }

    if (useRegisteredScan_ && hasScanCloud_) {
      pcl::PointCloud<pcl::PointXYZI> scanCloud;
      pcl::fromROSMsg(*latestScan_, scanCloud);
      analyzer_.appendObstacles(scanCloud, vx, vy, vz, vyaw, false);
    }
  }

  bool isGoalPathClear(float minDist) const {
    float checkDist = 0.0f;
    const float minRange = goalPathMinRange(minDist, &checkDist);
    return minRange >= checkDist;
  }

  float forwardHemisphereMinRange() const {
    const float forwardMaxRad = static_cast<float>(forwardMaxAngleDeg_ * kPi / 180.0);
    float minRange = static_cast<float>(scanRange_);
    for (int i = 0; i < binNum_; ++i) {
      const float angle = analyzer_.binAngle(i);
      if (std::abs(angle) <= forwardMaxRad) {
        minRange = std::min(minRange, analyzer_.freeRanges()[i]);
      }
    }
    return minRange;
  }

  bool isForwardClear(float minDist) const {
    return forwardHemisphereMinRange() >= minDist;
  }

  void lockExitBearingForward() {
    const float forwardMaxRad = static_cast<float>(forwardMaxAngleDeg_ * kPi / 180.0);
    if (lastEscapeGapValid_ && std::abs(lastEscapeBearing_) <= forwardMaxRad) {
      exitBearingVehicle_ = lastEscapeBearing_;
    } else {
      exitBearingVehicle_ = 0.0f;
    }
    hasExitBearing_ = true;
  }

  float selectCircumBearing(float goalBearing) const {
    const float forwardMaxRad = static_cast<float>(forwardMaxAngleDeg_ * kPi / 180.0);
    const float circumHalfRad = static_cast<float>(circumConeHalfDeg_ * kPi / 180.0);
    const float lateralRad = static_cast<float>(circumLateralDeg_ * kPi / 180.0);

    if (isGoalInForwardHemisphere(goalBearing)) {
      return analyzer_.findBestDirectionInForwardCone(goalBearing, goalBearing, circumHalfRad,
                                                      forwardMaxRad);
    }

    const float sideSign = circumSideSign_ != 0 ? static_cast<float>(circumSideSign_)
                                                  : (goalBearing >= 0.0f ? 1.0f : -1.0f);
    const float lateralCenter = sideSign * std::min(lateralRad, forwardMaxRad);

    return analyzer_.findBestDirectionInForwardCone(lateralCenter, lateralCenter, circumHalfRad,
                                                    forwardMaxRad);
  }

  float selectNavigateBearing(float goalBearing) const {
    const float forwardMaxRad = static_cast<float>(forwardMaxAngleDeg_ * kPi / 180.0);
    const float coneHalfRad = static_cast<float>(navigateConeHalfDeg_ * kPi / 180.0);

    if (!hasExitBearing_) {
      return analyzer_.findBestDirectionInForwardCone(0.0f, goalBearing,
                                                      std::min(coneHalfRad, forwardMaxRad),
                                                      forwardMaxRad);
    }

    const float goalFree = analyzer_.getFreeRangeAt(goalBearing);
    const bool goalLooksLikeCavity = goalFree >= static_cast<float>(scanRange_ * cavityFreeRatio_);
    const float backDiff = std::abs(normalizeAngle(goalBearing - exitBearingVehicle_));
    const float noReturnRad = static_cast<float>(noReturnAngleDeg_ * kPi / 180.0);
    const bool goalPointsBackward = backDiff > noReturnRad;
    const bool goalTooClose = goalFree < static_cast<float>(minSubgoalDist_);

    if (goalPointsBackward || goalLooksLikeCavity || goalTooClose) {
      return analyzer_.findBestDirectionInForwardCone(
          exitBearingVehicle_, goalBearing, std::min(coneHalfRad, forwardMaxRad), forwardMaxRad);
    }

    return analyzer_.findBestDirectionInForwardCone(
        exitBearingVehicle_, goalBearing, std::min(coneHalfRad, forwardMaxRad), forwardMaxRad);
  }

  bool isGoalInForwardHemisphere(float goalBearing) const {
    return std::abs(goalBearing) <=
           static_cast<float>(directNavigateMaxDeg_ * kPi / 180.0);
  }

  void publishWaypointMap(float xMap, float yMap, float zMap) {
    geometry_msgs::PointStamped waypoint;
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map";
    waypoint.point.x = xMap;
    waypoint.point.y = yMap;
    waypoint.point.z = zMap;
    pubWaypoint_.publish(waypoint);
  }

  void publishSpeed() {
    std_msgs::Float32 speedMsg;
    speedMsg.data = static_cast<float>(state_ == STATE_STUCK ? 0.0 : speed_);
    pubSpeed_.publish(speedMsg);
  }

  void publishState() {
    std_msgs::String stateMsg;
    stateMsg.data = stateToString(state_);
    pubState_.publish(stateMsg);
  }

  void publishSubgoalVehicleAngle(float angleVehicle, float freeRange) {
    float dist = static_cast<float>(subgoalDist_);
    dist = std::min(dist, static_cast<float>(maxSubgoalDist_));
    dist = std::max(dist, static_cast<float>(minSubgoalDist_));
    dist = std::min(dist, freeRange * 0.7f);
    dist = std::max(dist, static_cast<float>(minSubgoalDist_ * 0.5));

    const float xVehicle = dist * std::cos(angleVehicle);
    const float yVehicle = dist * std::sin(angleVehicle);
    float xMap = 0.0f;
    float yMap = 0.0f;
    vehicleToMap(xVehicle, yVehicle, static_cast<float>(vehicleX_), static_cast<float>(vehicleY_),
                 static_cast<float>(vehicleYaw_), &xMap, &yMap);
    publishWaypointMap(xMap, yMap, static_cast<float>(goalZ_));

    if (verboseLog_) {
      ROS_INFO_THROTTLE(logThrottleSec_,
                        "[polar_gap] SUBGOAL (%.2f, %.2f) | veh_angle=%.1fdeg dist=%.2f freeRange=%.2f",
                        xMap, yMap, radToDeg(angleVehicle), dist, freeRange);
    }
  }

  void publishStopWaypoint() {
    publishWaypointMap(static_cast<float>(vehicleX_), static_cast<float>(vehicleY_),
                       static_cast<float>(vehicleZ_));
  }

  void publishMarkers(const polar_gap_planner::GapSector& gap) {
    if (!publishMarkers_) {
      return;
    }

    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    const float cosYaw = std::cos(static_cast<float>(vehicleYaw_));
    const float sinYaw = std::sin(static_cast<float>(vehicleYaw_));

    visualization_msgs::Marker rays;
    rays.header.frame_id = "map";
    rays.header.stamp = ros::Time::now();
    rays.ns = "polar_rays";
    rays.id = 0;
    rays.type = visualization_msgs::Marker::LINE_LIST;
    rays.action = visualization_msgs::Marker::ADD;
    rays.scale.x = 0.02;
    rays.pose.orientation.w = 1.0;

    const int rayStep = std::max(1, binNum_ / 72);
    for (int i = 0; i < binNum_; i += rayStep) {
      const float angle = analyzer_.binAngle(i);
      const float range = analyzer_.freeRanges()[i];
      const float xVehicle = range * std::cos(angle);
      const float yVehicle = range * std::sin(angle);

      geometry_msgs::Point p0;
      p0.x = vehicleX_;
      p0.y = vehicleY_;
      p0.z = vehicleZ_ + 0.1;

      geometry_msgs::Point p1;
      p1.x = vehicleX_ + xVehicle * cosYaw - yVehicle * sinYaw;
      p1.y = vehicleY_ + xVehicle * sinYaw + yVehicle * cosYaw;
      p1.z = vehicleZ_ + 0.1;

      rays.points.push_back(p0);
      rays.points.push_back(p1);

      std_msgs::ColorRGBA color;
      if (range >= minFreeDistance_) {
        color.r = 0.1f;
        color.g = 0.9f;
        color.b = 0.2f;
        color.a = 0.8f;
      } else {
        color.r = 0.9f;
        color.g = 0.2f;
        color.b = 0.1f;
        color.a = 0.8f;
      }
      rays.colors.push_back(color);
      rays.colors.push_back(color);
    }
    array.markers.push_back(rays);

    if (gap.valid) {
      visualization_msgs::Marker sector;
      sector.header.frame_id = "map";
      sector.header.stamp = ros::Time::now();
      sector.ns = "selected_gap";
      sector.id = 1;
      sector.type = visualization_msgs::Marker::LINE_STRIP;
      sector.action = visualization_msgs::Marker::ADD;
      sector.scale.x = 0.05;
      sector.pose.orientation.w = 1.0;
      sector.color.r = 0.1f;
      sector.color.g = 0.5f;
      sector.color.b = 1.0f;
      sector.color.a = 1.0f;

      geometry_msgs::Point origin;
      origin.x = vehicleX_;
      origin.y = vehicleY_;
      origin.z = vehicleZ_ + 0.15;
      sector.points.push_back(origin);

      const float drawRange = std::min(static_cast<float>(maxSubgoalDist_), gap.freeRange);
      for (int step = 0; step <= 20; ++step) {
        const float t = static_cast<float>(step) / 20.0f;
        const float angle = gap.centerAngle - gap.widthAngle * 0.5f + t * gap.widthAngle;
        const float xVehicle = drawRange * std::cos(angle);
        const float yVehicle = drawRange * std::sin(angle);
        geometry_msgs::Point p;
        p.x = vehicleX_ + xVehicle * cosYaw - yVehicle * sinYaw;
        p.y = vehicleY_ + xVehicle * sinYaw + yVehicle * cosYaw;
        p.z = vehicleZ_ + 0.15;
        sector.points.push_back(p);
      }
      sector.points.push_back(origin);
      array.markers.push_back(sector);
    }

    pubMarkers_.publish(array);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!hasOdom_ || !hasObstacleCloud() || !hasGoal_) {
      if (verboseLog_) {
        ROS_WARN_THROTTLE(logThrottleSec_,
                          "[polar_gap] WAIT | odom=%d terrain=%d scan=%d goal=%d",
                          hasOdom_, hasTerrainCloud_, hasScanCloud_, hasGoal_);
      }
      return;
    }

    updateAnalyzerClouds();

    const float distGoal = distanceToGoal();
    if (distGoal < goalTolerance_) {
      logStateTransition(STATE_DONE, "reached final goal");
      state_ = STATE_DONE;
      publishStopWaypoint();
      publishSpeed();
      publishState();
      publishMarkers(polar_gap_planner::GapSector());
      if (verboseLog_) {
        ROS_WARN("[polar_gap] DONE | dist_goal=%.2f < tol=%.2f", distGoal, goalTolerance_);
      }
      return;
    }

    const float goalBearing = goalBearingVehicle();
    float navCheckDist = 0.0f;
    float escCheckDist = 0.0f;
    const float navMinRange = goalPathMinRange(static_cast<float>(navigateFreeDist_), &navCheckDist);
    const float escMinRange = goalPathMinRange(static_cast<float>(escapeFreeDist_), &escCheckDist);
    const bool directFreeNavigate = navMinRange >= navCheckDist;
    const bool directFreeEscape = escMinRange >= escCheckDist;
    const bool forwardClearEscape =
        isForwardClear(static_cast<float>(escapeFreeDist_));
    float circumNavCheckDist = 0.0f;
    const float circumNavMinRange =
        goalPathMinRange(static_cast<float>(circumNavigateFreeDist_), &circumNavCheckDist);
    const float goalDirFree = analyzer_.getFreeRangeAt(goalBearing);
    const bool canNavigateFromCircum =
        goalDirFree >= static_cast<float>(minSubgoalDist_) &&
        (circumNavMinRange >= circumNavCheckDist ||
         goalDirFree >= static_cast<float>(circumNavigateFreeDist_));

    const int prevState = state_;
    const float goalFree = analyzer_.getFreeRangeAt(goalBearing);
    const bool goalLooksLikeCavity = goalFree >= static_cast<float>(scanRange_ * cavityFreeRatio_);
    const float noReturnRad = static_cast<float>(noReturnAngleDeg_ * kPi / 180.0);
    const float backDiffFromExit = std::abs(normalizeAngle(goalBearing - lastEscapeBearing_));
    const bool goalInFront = isGoalInForwardHemisphere(goalBearing);
    const bool exitedTrap = directFreeEscape && lastEscapeGapValid_;

    if (state_ == STATE_ESCAPE || state_ == STATE_STUCK) {
      if (directFreeNavigate && goalInFront && !goalLooksLikeCavity &&
          (!lastEscapeGapValid_ || backDiffFromExit <= noReturnRad)) {
        state_ = STATE_NAVIGATE;
      } else if (exitedTrap && !goalInFront) {
        state_ = STATE_CIRCUM;
      }
    } else if (state_ == STATE_CIRCUM) {
      if (!forwardClearEscape) {
        ++circumBlockedTicks_;
        if (circumBlockedTicks_ >= circumEscapeBlockedTicks_) {
          state_ = STATE_ESCAPE;
        }
      } else {
        circumBlockedTicks_ = 0;
      }
      if (goalInFront && (directFreeNavigate || canNavigateFromCircum)) {
        state_ = STATE_NAVIGATE;
      }
    } else if (state_ == STATE_NAVIGATE) {
      if (!forwardClearEscape) {
        state_ = STATE_ESCAPE;
      } else if (!goalInFront) {
        state_ = STATE_CIRCUM;
      } else if (hasExitBearing_ &&
                 std::abs(normalizeAngle(goalBearing - exitBearingVehicle_)) > noReturnRad) {
        state_ = STATE_CIRCUM;
      }
    }

    if (state_ != prevState) {
      if (state_ == STATE_NAVIGATE) {
        if (!hasExitBearing_) {
          lockExitBearingForward();
        }
        logStateTransition(state_, "goal in front, direct navigate");
        if (verboseLog_) {
          ROS_WARN("[polar_gap] EXIT_BEARING locked %.1fdeg (goal %.1fdeg)",
                   radToDeg(exitBearingVehicle_), radToDeg(goalBearing));
        }
      } else if (state_ == STATE_CIRCUM) {
        if (!hasExitBearing_) {
          lockExitBearingForward();
        }
        if (circumSideSign_ == 0) {
          circumSideSign_ = goalBearing >= 0.0f ? 1 : -1;
        }
        circumBlockedTicks_ = 0;
        logStateTransition(state_, "goal behind, circumnavigate around trap");
      } else if (state_ == STATE_ESCAPE) {
        if (prevState == STATE_CIRCUM) {
          logStateTransition(state_, "forward blocked, resume forward gap search");
        } else {
          logStateTransition(state_, "goal path blocked, search 360 gap");
        }
      }
    }

    polar_gap_planner::GapSector gap;
    if (state_ == STATE_NAVIGATE) {
      const float navBearing = selectNavigateBearing(goalBearing);
      publishSubgoalVehicleAngle(navBearing, analyzer_.getFreeRangeAt(navBearing));
      gap.centerAngle = navBearing;
      gap.widthAngle = static_cast<float>(goalCheckHalfWidthDeg_ * kPi / 180.0 * 2.0);
      gap.freeRange = analyzer_.getFreeRangeAt(navBearing);
      gap.valid = true;
      if (verboseLog_) {
        ROS_INFO_THROTTLE(logThrottleSec_,
                          "[polar_gap] NAV | goal=%.1fdeg -> nav=%.1fdeg (forward only)",
                          radToDeg(goalBearing), radToDeg(navBearing));
      }
    } else if (state_ == STATE_CIRCUM) {
      const float circumBearing = selectCircumBearing(goalBearing);
      publishSubgoalVehicleAngle(circumBearing, analyzer_.getFreeRangeAt(circumBearing));
      gap.centerAngle = circumBearing;
      gap.widthAngle = static_cast<float>(circumConeHalfDeg_ * kPi / 180.0 * 2.0);
      gap.freeRange = analyzer_.getFreeRangeAt(circumBearing);
      gap.valid = true;
      if (verboseLog_) {
        ROS_INFO_THROTTLE(logThrottleSec_,
                          "[polar_gap] CIRCUM | goal=%.1fdeg -> skirt=%.1fdeg side=%d lateral=%.1fdeg exit=%.1fdeg",
                          radToDeg(goalBearing), radToDeg(circumBearing), circumSideSign_,
                          radToDeg(static_cast<float>(circumSideSign_) *
                                   static_cast<float>(circumLateralDeg_ * kPi / 180.0)),
                          radToDeg(exitBearingVehicle_));
      }
    } else {
      const float forwardMaxRad = static_cast<float>(forwardMaxAngleDeg_ * kPi / 180.0);
      if (hasExitBearing_) {
        gap = analyzer_.findBestGapSectorInForwardCone(0.0f, forwardMaxRad, false);
      } else {
        gap = analyzer_.findWidestFreeSector(goalBearing, false);
      }
      if (gap.valid) {
        lastEscapeBearing_ = gap.centerAngle;
        lastEscapeGapValid_ = true;
        if (prevState == STATE_STUCK) {
          logStateTransition(STATE_ESCAPE, "valid gap found, resume escape");
        }
        state_ = STATE_ESCAPE;
        publishSubgoalVehicleAngle(gap.centerAngle, gap.freeRange);
        if (verboseLog_) {
          ROS_INFO_THROTTLE(logThrottleSec_,
                            "[polar_gap] GAP | center=%.1fdeg width=%.1fdeg freeRange=%.2f "
                            "(goalBearing=%.1fdeg)",
                            radToDeg(gap.centerAngle), radToDeg(gap.widthAngle), gap.freeRange,
                            radToDeg(goalBearing));
        }
      } else {
        logStateTransition(STATE_STUCK, "no gap wider than min_gap_width_deg");
        state_ = STATE_STUCK;
        publishStopWaypoint();
        if (verboseLog_) {
          ROS_WARN_THROTTLE(logThrottleSec_,
                            "[polar_gap] STUCK | no valid gap >= %.1fdeg with gap_free=%.2f | "
                            "gap_blocked=%d/%d",
                            minGapWidthDeg_, gapFreeDistance_, countGapBlockedBins(), binNum_);
        }
      }
    }

    if (verboseLog_) {
      ROS_INFO_THROTTLE(logThrottleSec_,
                        "[polar_gap] TICK | state=%s | veh=(%.2f,%.2f) yaw=%.1fdeg | "
                        "goal=(%.2f,%.2f) dist=%.2f bearing=%.1fdeg | "
                        "goal_range nav=%.2f/%.2f esc=%.2f/%.2f fwd=%.2f | obs_bins=%d gap_blocked=%d | "
                        "terrain=%d scan=%d",
                        stateToString(state_).c_str(), vehicleX_, vehicleY_,
                        radToDeg(static_cast<float>(vehicleYaw_)), goalX_, goalY_, distGoal,
                        radToDeg(goalBearing), navMinRange, navCheckDist, escMinRange, escCheckDist,
                        forwardHemisphereMinRange(),
                        countUpdatedBins(), countGapBlockedBins(), hasTerrainCloud_, hasScanCloud_);
    }

    publishSpeed();
    publishState();
    publishMarkers(gap);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  polar_gap_planner::PolarAnalyzer analyzer_;

  ros::Subscriber subOdom_;
  ros::Subscriber subTerrain_;
  ros::Subscriber subScan_;
  ros::Subscriber subGoalPose_;
  ros::Subscriber subGoalPoint_;
  ros::Subscriber subNavGoal_;
  ros::Publisher pubWaypoint_;
  ros::Publisher pubSpeed_;
  ros::Publisher pubState_;
  ros::Publisher pubMarkers_;
  ros::Timer timer_;

  sensor_msgs::PointCloud2ConstPtr latestTerrain_;
  sensor_msgs::PointCloud2ConstPtr latestScan_;

  double vehicleX_;
  double vehicleY_;
  double vehicleZ_;
  double vehicleRoll_;
  double vehiclePitch_;
  double vehicleYaw_;

  double goalX_;
  double goalY_;
  double goalZ_;

  double scanRange_;
  int binNum_;
  double obstacleHeightThre_;
  double minObstacleZ_;
  double maxObstacleZ_;
  double minFreeDistance_;
  double gapFreeDistance_;
  double minGapWidthDeg_;
  double subgoalDist_;
  double minSubgoalDist_;
  double maxSubgoalDist_;
  double goalTolerance_;
  double navigateFreeDist_;
  double escapeFreeDist_;
  double replanRate_;
  double speed_;
  double goalCheckHalfWidthDeg_;
  double navigateConeHalfDeg_;
  double noReturnAngleDeg_;
  double cavityFreeRatio_;
  double forwardMaxAngleDeg_;
  double directNavigateMaxDeg_;
  double skirtAngleDeg_;
  double circumConeHalfDeg_;
  double circumLateralDeg_;
  double circumNavigateFreeDist_;
  int circumEscapeBlockedTicks_;

  bool useTerrainMap_;
  bool useRegisteredScan_;
  bool publishMarkers_;
  bool hasOdom_;
  bool hasTerrainCloud_;
  bool hasScanCloud_;
  bool hasGoal_;

  std::string terrainTopic_;
  std::string scanTopic_;

  int state_;
  int lastLoggedState_;
  bool verboseLog_;
  double logThrottleSec_;

  bool hasExitBearing_;
  float exitBearingVehicle_;
  float lastEscapeBearing_;
  bool lastEscapeGapValid_;
  int circumSideSign_;
  int circumBlockedTicks_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "polar_gap_planner");
  PolarGapPlannerNode node;
  ros::spin();
  return 0;
}
