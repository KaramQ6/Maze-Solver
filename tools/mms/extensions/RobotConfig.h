#pragma once

#include <QSize>
#include <QString>

#include "Mouse.h"

namespace mms {

struct RobotStart {
  int x = 0;
  int y = 0;
  SemiDirection heading = SemiDirection::NORTH;

  SemiPosition semiPosition() const { return {2 * x + 1, 2 * y + 1}; }
  QChar headingChar() const;
};

struct MotionTiming {
  // Illustrative defaults, not measurements of the team's robot.
  double searchCellSeconds = 0.5;
  double speedCellSeconds = 0.3;
  double turn90Seconds = 0.25;
  bool calibrated = false;
};

RobotStart loadRobotStart(const QString &mazePath, QSize mazeSize);
void saveRobotStart(const QString &mazePath, const RobotStart &start);
MotionTiming loadMotionTiming();
void saveMotionTiming(const MotionTiming &timing);

}  // namespace mms
