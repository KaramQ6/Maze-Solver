#include "RobotDialogs.h"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace mms {
namespace {

void addButtons(QVBoxLayout *layout, QDialog *dialog) {
  auto buttons = new QDialogButtonBox(QDialogButtonBox::Apply | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Apply)->setObjectName("applyRobotSettings");
  QObject::connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked,
                   dialog, &QDialog::accept);
  QObject::connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
  layout->addWidget(buttons);
}

QDoubleSpinBox *timingField(const QString &name, double value) {
  auto field = new QDoubleSpinBox();
  field->setObjectName(name);
  field->setRange(0.01, 60.0);
  field->setDecimals(3);
  field->setSingleStep(0.01);
  field->setSuffix(" s");
  field->setValue(value);
  return field;
}

}  // namespace

StartDialog::StartDialog(const RobotStart &start, QSize mazeSize, QWidget *parent)
    : QDialog(parent), m_x(new QSpinBox()), m_y(new QSpinBox()),
      m_heading(new QComboBox()) {
  setWindowTitle("Set Start");
  setObjectName("startDialog");
  auto layout = new QVBoxLayout(this);
  auto hint = new QLabel("Coordinates are zero-based: (0, 0) is bottom-left.\n"
                         "Applying a new start restarts an active run.");
  hint->setWordWrap(true);
  layout->addWidget(hint);
  auto form = new QFormLayout();
  m_x->setObjectName("startX");
  m_y->setObjectName("startY");
  m_heading->setObjectName("startHeading");
  m_x->setRange(0, mazeSize.width() - 1);
  m_y->setRange(0, mazeSize.height() - 1);
  m_x->setValue(start.x);
  m_y->setValue(start.y);
  for (auto heading : {SemiDirection::NORTH, SemiDirection::EAST,
                       SemiDirection::SOUTH, SemiDirection::WEST}) {
    QString name;
    switch (heading) {
      case SemiDirection::NORTH: name = "North"; break;
      case SemiDirection::EAST: name = "East"; break;
      case SemiDirection::SOUTH: name = "South"; break;
      default: name = "West"; break;
    }
    m_heading->addItem(name, static_cast<int>(heading));
  }
  m_heading->setCurrentIndex(m_heading->findData(static_cast<int>(start.heading)));
  form->addRow("&X (column)", m_x);
  form->addRow("&Y (from bottom)", m_y);
  form->addRow("&Heading", m_heading);
  layout->addLayout(form);
  addButtons(layout, this);
}

RobotStart StartDialog::selectedStart() const {
  RobotStart start;
  start.x = m_x->value();
  start.y = m_y->value();
  start.heading = static_cast<SemiDirection>(m_heading->currentData().toInt());
  return start;
}

TimingDialog::TimingDialog(const MotionTiming &timing, QWidget *parent)
    : QDialog(parent),
      m_search(timingField("searchCellSeconds", timing.searchCellSeconds)),
      m_speed(timingField("speedCellSeconds", timing.speedCellSeconds)),
      m_turn(timingField("turn90Seconds", timing.turn90Seconds)),
      m_calibrated(new QCheckBox("Values measured on the robot")) {
  setWindowTitle("Robot Speed - ESP32-C3");
  setObjectName("timingDialog");
  auto layout = new QVBoxLayout(this);
  auto hint = new QLabel("ESP32-C3 is the controller, not a fixed driving speed.\n"
                         "Measure one cell-center move (192 mm) and a 90-degree turn.\n"
                         "Changes apply to the next movement; playback runs at real time (1x).");
  hint->setWordWrap(true);
  layout->addWidget(hint);
  auto form = new QFormLayout();
  form->addRow("&Search: one cell", m_search);
  form->addRow("S&peed run: one cell", m_speed);
  form->addRow("&Turn: 90 degrees", m_turn);
  layout->addLayout(form);
  auto velocity = new QLabel();
  velocity->setObjectName("robotVelocity");
  velocity->setWordWrap(true);
  auto updateVelocity = [=]() {
    velocity->setText(QString("Average speed: search %1 m/s | speed run %2 m/s")
        .arg(0.192 / m_search->value(), 0, 'f', 3)
        .arg(0.192 / m_speed->value(), 0, 'f', 3));
  };
  connect(m_search, &QDoubleSpinBox::valueChanged, this, updateVelocity);
  connect(m_speed, &QDoubleSpinBox::valueChanged, this, updateVelocity);
  updateVelocity();
  layout->addWidget(velocity);
  m_calibrated->setObjectName("robotCalibrated");
  m_calibrated->setChecked(timing.calibrated);
  layout->addWidget(m_calibrated);
  addButtons(layout, this);
}

MotionTiming TimingDialog::selectedTiming() const {
  MotionTiming timing;
  timing.searchCellSeconds = m_search->value();
  timing.speedCellSeconds = m_speed->value();
  timing.turn90Seconds = m_turn->value();
  timing.calibrated = m_calibrated->isChecked();
  return timing;
}

}  // namespace mms
