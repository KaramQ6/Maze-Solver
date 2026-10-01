#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QSpinBox>

#include "RobotConfig.h"

namespace mms {

class StartDialog : public QDialog {
 public:
  StartDialog(const RobotStart &start, QSize mazeSize, QWidget *parent);
  RobotStart selectedStart() const;

 private:
  QSpinBox *m_x;
  QSpinBox *m_y;
  QComboBox *m_heading;
};

class TimingDialog : public QDialog {
 public:
  TimingDialog(const MotionTiming &timing, QWidget *parent);
  MotionTiming selectedTiming() const;

 private:
  QDoubleSpinBox *m_search;
  QDoubleSpinBox *m_speed;
  QDoubleSpinBox *m_turn;
  QCheckBox *m_calibrated;
};

}  // namespace mms
