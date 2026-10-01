#include "RobotConfig.h"

#include <QCryptographicHash>
#include <QtMath>

#include "Settings.h"

namespace mms {
namespace {

QString startGroup(const QString &mazePath) {
  auto hash = QCryptographicHash::hash(mazePath.toUtf8(), QCryptographicHash::Sha256);
  return "mmrc26-starts/" + QString::fromLatin1(hash.toHex());
}

double seconds(const QString &key, double fallback, bool &valid) {
  bool ok = false;
  double value = Settings::get()->value("mmrc26-timing", key).toDouble(&ok);
  bool accepted = ok && qIsFinite(value) && 0.01 <= value && value <= 60.0;
  valid = valid && accepted;
  return accepted ? value : fallback;
}

}  // namespace

QChar RobotStart::headingChar() const {
  return CHAR_TO_DIRECTION().key(SEMI_TO_CARDINAL().value(heading));
}

RobotStart loadRobotStart(const QString &mazePath, QSize mazeSize) {
  RobotStart start;
  QString group = startGroup(mazePath);
  bool xOk = false;
  bool yOk = false;
  int x = Settings::get()->value(group, "x").toInt(&xOk);
  int y = Settings::get()->value(group, "y").toInt(&yOk);
  QString heading = Settings::get()->value(group, "heading");
  if (xOk && yOk && 0 <= x && x < mazeSize.width() &&
      0 <= y && y < mazeSize.height() && heading.size() == 1 &&
      CHAR_TO_DIRECTION().contains(heading.at(0))) {
    start.x = x;
    start.y = y;
    start.heading = SEMI_TO_CARDINAL().key(CHAR_TO_DIRECTION().value(heading.at(0)));
  }
  return start;
}

void saveRobotStart(const QString &mazePath, const RobotStart &start) {
  QString group = startGroup(mazePath);
  Settings::get()->update(group, "x", QString::number(start.x));
  Settings::get()->update(group, "y", QString::number(start.y));
  Settings::get()->update(group, "heading", QString(start.headingChar()));
}

MotionTiming loadMotionTiming() {
  MotionTiming timing;
  bool valid = true;
  timing.searchCellSeconds = seconds("search-cell-seconds", timing.searchCellSeconds, valid);
  timing.speedCellSeconds = seconds("speed-cell-seconds", timing.speedCellSeconds, valid);
  timing.turn90Seconds = seconds("turn-90-seconds", timing.turn90Seconds, valid);
  timing.calibrated = valid && Settings::get()->value("mmrc26-timing", "calibrated") == "true";
  return timing;
}

void saveMotionTiming(const MotionTiming &timing) {
  auto settings = Settings::get();
  settings->update("mmrc26-timing", "search-cell-seconds", QString::number(timing.searchCellSeconds));
  settings->update("mmrc26-timing", "speed-cell-seconds", QString::number(timing.speedCellSeconds));
  settings->update("mmrc26-timing", "turn-90-seconds", QString::number(timing.turn90Seconds));
  settings->update("mmrc26-timing", "calibrated", timing.calibrated ? "true" : "false");
}

}  // namespace mms
