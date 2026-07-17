#ifndef POLAR_GAP_PLANNER_POLAR_ANALYZER_H
#define POLAR_GAP_PLANNER_POLAR_ANALYZER_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace polar_gap_planner {

struct GapSector {
  float centerAngle;
  float widthAngle;
  float freeRange;
  int startBin;
  int endBin;
  bool valid;

  GapSector()
      : centerAngle(0.0f),
        widthAngle(0.0f),
        freeRange(0.0f),
        startBin(0),
        endBin(0),
        valid(false) {}
};

class PolarAnalyzer {
 public:
  PolarAnalyzer();

  void setScanRange(float scanRange);
  void setBinNum(int binNum);
  void setMinFreeDistance(float minFreeDistance);
  void setGapFreeDistance(float gapFreeDistance);
  void setMinGapWidthDeg(float minGapWidthDeg);
  void setObstacleHeightThre(float obstacleHeightThre);
  void setMinObstacleZ(float minObstacleZ);
  void setMaxObstacleZ(float maxObstacleZ);
  void setUseTerrainMap(bool useTerrainMap);

  void reset();
  void updateObstacles(const pcl::PointCloud<pcl::PointXYZI>& cloud, float vehicleX,
                       float vehicleY, float vehicleZ, float vehicleYaw);
  void appendObstacles(const pcl::PointCloud<pcl::PointXYZI>& cloud, float vehicleX,
                       float vehicleY, float vehicleZ, float vehicleYaw, bool useIntensityFilter);

  GapSector findBestGapSector(float preferredAngleRad, bool preferGoalDirection) const;
  GapSector findBestGapSectorInForwardCone(float preferredAngleRad, float forwardMaxRad,
                                           bool preferGoalDirection) const;
  GapSector findWidestFreeSector(float preferredAngleRad, bool preferGoalDirection = true) const;
  float findBestDirectionInCone(float coneCenterRad, float goalBearingRad,
                                float coneHalfRad) const;
  float findBestDirectionInForwardCone(float coneCenterRad, float goalBearingRad,
                                       float coneHalfRad, float forwardMaxRad) const;
  float getFreeRangeAt(float angleVehicleRad) const;
  bool isDirectionFree(float angleVehicleRad, float minDist) const;

  const std::vector<float>& freeRanges() const { return freeRange_; }
  int binNum() const { return binNum_; }
  float binAngle(int bin) const;
  float binWidthRad() const;

 private:
  int angleToBin(float angleRad) const;
  float normalizeAngle(float angleRad) const;

  float scanRange_;
  int binNum_;
  float minFreeDistance_;
  float gapFreeDistance_;
  float minGapWidthRad_;
  float obstacleHeightThre_;
  float minObstacleZ_;
  float maxObstacleZ_;
  bool useTerrainMap_;

  std::vector<float> freeRange_;
  std::vector<bool> binUpdated_;
};

}  // namespace polar_gap_planner

#endif
