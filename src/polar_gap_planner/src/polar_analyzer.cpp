#include "polar_gap_planner/polar_analyzer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace polar_gap_planner {

namespace {
const float kPi = 3.1415926f;
}

PolarAnalyzer::PolarAnalyzer()
    : scanRange_(5.0f),
      binNum_(360),
      minFreeDistance_(1.2f),
      gapFreeDistance_(2.2f),
      minGapWidthRad_(30.0f * kPi / 180.0f),
      obstacleHeightThre_(0.15f),
      minObstacleZ_(0.15f),
      maxObstacleZ_(1.5f),
      useTerrainMap_(true) {
  reset();
}

void PolarAnalyzer::setScanRange(float scanRange) { scanRange_ = scanRange; }

void PolarAnalyzer::setBinNum(int binNum) {
  binNum_ = std::max(36, binNum);
  reset();
}

void PolarAnalyzer::setMinFreeDistance(float minFreeDistance) {
  minFreeDistance_ = minFreeDistance;
}

void PolarAnalyzer::setGapFreeDistance(float gapFreeDistance) {
  gapFreeDistance_ = gapFreeDistance;
}

void PolarAnalyzer::setMinGapWidthDeg(float minGapWidthDeg) {
  minGapWidthRad_ = minGapWidthDeg * kPi / 180.0f;
}

void PolarAnalyzer::setObstacleHeightThre(float obstacleHeightThre) {
  obstacleHeightThre_ = obstacleHeightThre;
}

void PolarAnalyzer::setMinObstacleZ(float minObstacleZ) { minObstacleZ_ = minObstacleZ; }

void PolarAnalyzer::setMaxObstacleZ(float maxObstacleZ) { maxObstacleZ_ = maxObstacleZ; }

void PolarAnalyzer::setUseTerrainMap(bool useTerrainMap) { useTerrainMap_ = useTerrainMap; }

void PolarAnalyzer::reset() {
  freeRange_.assign(binNum_, scanRange_);
  binUpdated_.assign(binNum_, false);
}

float PolarAnalyzer::normalizeAngle(float angleRad) const {
  while (angleRad > kPi) {
    angleRad -= 2.0f * kPi;
  }
  while (angleRad < -kPi) {
    angleRad += 2.0f * kPi;
  }
  return angleRad;
}

float PolarAnalyzer::binWidthRad() const { return 2.0f * kPi / static_cast<float>(binNum_); }

float PolarAnalyzer::binAngle(int bin) const {
  return -kPi + (static_cast<float>(bin) + 0.5f) * binWidthRad();
}

int PolarAnalyzer::angleToBin(float angleRad) const {
  angleRad = normalizeAngle(angleRad);
  float shifted = angleRad + kPi;
  int bin = static_cast<int>(shifted / binWidthRad());
  if (bin >= binNum_) {
    bin = binNum_ - 1;
  }
  if (bin < 0) {
    bin = 0;
  }
  return bin;
}

void PolarAnalyzer::updateObstacles(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                    float vehicleX, float vehicleY, float vehicleZ,
                                    float vehicleYaw) {
  reset();
  appendObstacles(cloud, vehicleX, vehicleY, vehicleZ, vehicleYaw, useTerrainMap_);
}

void PolarAnalyzer::appendObstacles(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                    float vehicleX, float vehicleY, float vehicleZ,
                                    float vehicleYaw, bool useIntensityFilter) {
  const float cosYaw = std::cos(vehicleYaw);
  const float sinYaw = std::sin(vehicleYaw);

  for (const auto& point : cloud.points) {
    const float dx = point.x - vehicleX;
    const float dy = point.y - vehicleY;
    const float dz = point.z - vehicleZ;

    if (useIntensityFilter) {
      if (point.intensity <= obstacleHeightThre_) {
        continue;
      }
    } else if (dz < minObstacleZ_ || dz > maxObstacleZ_) {
      continue;
    }

    const float xVehicle = dx * cosYaw + dy * sinYaw;
    const float yVehicle = -dx * sinYaw + dy * cosYaw;
    const float range = std::sqrt(xVehicle * xVehicle + yVehicle * yVehicle);

    if (range < 0.05f || range > scanRange_) {
      continue;
    }

    const float angle = std::atan2(yVehicle, xVehicle);
    const int bin = angleToBin(angle);

    if (!binUpdated_[bin] || range < freeRange_[bin]) {
      freeRange_[bin] = range;
      binUpdated_[bin] = true;
    }
  }
}

float PolarAnalyzer::getFreeRangeAt(float angleVehicleRad) const {
  return freeRange_[angleToBin(angleVehicleRad)];
}

bool PolarAnalyzer::isDirectionFree(float angleVehicleRad, float minDist) const {
  return getFreeRangeAt(angleVehicleRad) >= minDist;
}

GapSector PolarAnalyzer::findBestGapSector(float preferredAngleRad,
                                           bool preferGoalDirection) const {
  GapSector bestGap;
  preferredAngleRad = normalizeAngle(preferredAngleRad);

  int bestLen = 0;
  float bestScore = -1.0f;
  float bestGoalDiff = std::numeric_limits<float>::max();

  for (int start = 0; start < binNum_; ++start) {
    if (freeRange_[start % binNum_] < gapFreeDistance_) {
      continue;
    }

    int end = start;
    float minRangeInSector = freeRange_[start % binNum_];

    while (end < start + binNum_ && freeRange_[end % binNum_] >= gapFreeDistance_) {
      minRangeInSector = std::min(minRangeInSector, freeRange_[end % binNum_]);
      ++end;
    }

    int len = end - start;
    if (len > binNum_) {
      len = binNum_;
    }
    if (len <= 0) {
      continue;
    }

    const int startBin = start % binNum_;
    const int endBin = (start + len - 1) % binNum_;
    const float widthAngle = static_cast<float>(len) * binWidthRad();

    if (widthAngle + 1e-4f < minGapWidthRad_) {
      continue;
    }

    const int centerBin = (startBin + len / 2) % binNum_;
    const float centerAngle = binAngle(centerBin);
    const float goalDiff = std::abs(normalizeAngle(centerAngle - preferredAngleRad));
    const float score = minRangeInSector * widthAngle;

    const bool betterScore = score > bestScore + 1e-4f;
    const bool sameScoreBetterGoal =
        preferGoalDirection && (std::abs(score - bestScore) <= 1e-4f) &&
        (goalDiff + 1e-4f < bestGoalDiff);
    const bool sameScoreSameGoalWider =
        preferGoalDirection && (std::abs(score - bestScore) <= 1e-4f) &&
        (std::abs(goalDiff - bestGoalDiff) <= 1e-4f) && (len > bestLen);

    if (betterScore || sameScoreBetterGoal || sameScoreSameGoalWider) {
      bestScore = score;
      bestLen = len;
      bestGoalDiff = goalDiff;

      bestGap.valid = true;
      bestGap.startBin = startBin;
      bestGap.endBin = endBin;
      bestGap.widthAngle = widthAngle;
      bestGap.centerAngle = centerAngle;
      bestGap.freeRange = minRangeInSector;
    }
  }

  // Full-circle false free: pick the direction with the largest clearance.
  if (!bestGap.valid || bestLen > static_cast<int>(binNum_ * 0.85f)) {
    int maxBin = 0;
    float maxRange = -1.0f;
    for (int i = 0; i < binNum_; ++i) {
      if (freeRange_[i] > maxRange) {
        maxRange = freeRange_[i];
        maxBin = i;
      }
    }

    if (maxRange >= minFreeDistance_) {
      bestGap.valid = true;
      bestGap.startBin = maxBin;
      bestGap.endBin = maxBin;
      bestGap.widthAngle = binWidthRad();
      bestGap.centerAngle = binAngle(maxBin);
      bestGap.freeRange = maxRange;
    }
  }

  return bestGap;
}

GapSector PolarAnalyzer::findBestGapSectorInForwardCone(float preferredAngleRad,
                                                        float forwardMaxRad,
                                                        bool preferGoalDirection) const {
  GapSector bestGap;
  preferredAngleRad = normalizeAngle(preferredAngleRad);

  int bestLen = 0;
  float bestScore = -1.0f;
  float bestGoalDiff = std::numeric_limits<float>::max();

  for (int start = 0; start < binNum_; ++start) {
    const int startIdx = start % binNum_;
    const float startAngle = binAngle(startIdx);
    if (std::abs(startAngle) > forwardMaxRad) {
      continue;
    }
    if (freeRange_[startIdx] < gapFreeDistance_) {
      continue;
    }

    int end = start;
    float minRangeInSector = freeRange_[startIdx];

    while (end < start + binNum_) {
      const int endIdx = end % binNum_;
      const float endAngle = binAngle(endIdx);
      if (std::abs(endAngle) > forwardMaxRad || freeRange_[endIdx] < gapFreeDistance_) {
        break;
      }
      minRangeInSector = std::min(minRangeInSector, freeRange_[endIdx]);
      ++end;
    }

    int len = end - start;
    if (len <= 0) {
      continue;
    }

    const int startBin = startIdx;
    const int endBin = (start + len - 1) % binNum_;
    const float widthAngle = static_cast<float>(len) * binWidthRad();

    if (widthAngle + 1e-4f < minGapWidthRad_) {
      continue;
    }

    const int centerBin = (startBin + len / 2) % binNum_;
    const float centerAngle = binAngle(centerBin);
    const float goalDiff = std::abs(normalizeAngle(centerAngle - preferredAngleRad));
    const float score = minRangeInSector * widthAngle;

    const bool betterScore = score > bestScore + 1e-4f;
    const bool sameScoreBetterGoal =
        preferGoalDirection && (std::abs(score - bestScore) <= 1e-4f) &&
        (goalDiff + 1e-4f < bestGoalDiff);
    const bool sameScoreSameGoalWider =
        preferGoalDirection && (std::abs(score - bestScore) <= 1e-4f) &&
        (std::abs(goalDiff - bestGoalDiff) <= 1e-4f) && (len > bestLen);

    if (betterScore || sameScoreBetterGoal || sameScoreSameGoalWider) {
      bestScore = score;
      bestLen = len;
      bestGoalDiff = goalDiff;

      bestGap.valid = true;
      bestGap.startBin = startBin;
      bestGap.endBin = endBin;
      bestGap.widthAngle = widthAngle;
      bestGap.centerAngle = centerAngle;
      bestGap.freeRange = minRangeInSector;
    }
  }

  if (!bestGap.valid) {
    float maxRange = -1.0f;
    int maxBin = 0;
    for (int i = 0; i < binNum_; ++i) {
      const float angle = binAngle(i);
      if (std::abs(angle) > forwardMaxRad) {
        continue;
      }
      if (freeRange_[i] > maxRange) {
        maxRange = freeRange_[i];
        maxBin = i;
      }
    }

    if (maxRange >= minFreeDistance_) {
      bestGap.valid = true;
      bestGap.startBin = maxBin;
      bestGap.endBin = maxBin;
      bestGap.widthAngle = binWidthRad();
      bestGap.centerAngle = binAngle(maxBin);
      bestGap.freeRange = maxRange;
    }
  }

  return bestGap;
}

GapSector PolarAnalyzer::findWidestFreeSector(float preferredAngleRad,
                                              bool preferGoalDirection) const {
  return findBestGapSector(preferredAngleRad, preferGoalDirection);
}

float PolarAnalyzer::findBestDirectionInCone(float coneCenterRad, float goalBearingRad,
                                             float coneHalfRad) const {
  return findBestDirectionInForwardCone(coneCenterRad, goalBearingRad, coneHalfRad,
                                      static_cast<float>(kPi));
}

float PolarAnalyzer::findBestDirectionInForwardCone(float coneCenterRad, float goalBearingRad,
                                                    float coneHalfRad,
                                                    float forwardMaxRad) const {
  coneCenterRad = normalizeAngle(coneCenterRad);
  goalBearingRad = normalizeAngle(goalBearingRad);

  float bestAngle = 0.0f;
  float bestScore = -1.0f;
  bool found = false;

  for (int i = 0; i < binNum_; ++i) {
    const float angle = binAngle(i);
    if (std::abs(angle) > forwardMaxRad) {
      continue;
    }

    const float coneDiff = std::abs(normalizeAngle(angle - coneCenterRad));
    if (coneDiff > coneHalfRad) {
      continue;
    }

    const float goalDiff = std::abs(normalizeAngle(angle - goalBearingRad));
    const float goalWeight = 1.0f - goalDiff / static_cast<float>(kPi);
    const float score = freeRange_[i] * goalWeight;

    if (score > bestScore) {
      bestScore = score;
      bestAngle = angle;
      found = true;
    }
  }

  if (!found) {
    bestAngle = coneCenterRad;
    if (bestAngle > forwardMaxRad) {
      bestAngle = forwardMaxRad;
    } else if (bestAngle < -forwardMaxRad) {
      bestAngle = -forwardMaxRad;
    }
  }

  return bestAngle;
}

}  // namespace polar_gap_planner
