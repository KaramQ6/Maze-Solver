#include <QApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QSettings>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>

#include "ColorManager.h"
#include "Dimensions.h"
#include "RobotConfig.h"
#include "RobotDialogs.h"
#include "Settings.h"
#include "SettingsMisc.h"
#include "SettingsMouseAlgos.h"
#include "Window.h"

using namespace mms;

class NativeChecks : public QObject {
  Q_OBJECT

 private:
  QString root;
  QString snapshots;
  QTemporaryDir *settingsDirectory = nullptr;
  Window *window = nullptr;

  QPushButton *button(const QString &name) {
    return window->findChild<QPushButton *>(name);
  }

  QString output() {
    return window->findChild<QPlainTextEdit *>("algorithmOutput")->toPlainText();
  }

  double measured(const QString &label) {
    auto match = QRegularExpression(label + " ([0-9.]+)").match(output());
    return match.hasMatch() ? match.captured(1).toDouble() : -1.0;
  }

  void editStart(int x, int y, SemiDirection heading = SemiDirection::EAST) {
    QTimer::singleShot(30, this, [=]() {
      auto dialog = window->findChild<QDialog *>("startDialog");
      if (!dialog) {
        QFAIL("Start dialog did not open");
      }
      dialog->findChild<QSpinBox *>("startX")->setValue(x);
      dialog->findChild<QSpinBox *>("startY")->setValue(y);
      auto compass = dialog->findChild<QComboBox *>("startHeading");
      compass->setCurrentIndex(compass->findData(static_cast<int>(heading)));
      dialog->grab().save(snapshots + "/set-start.png");
      dialog->findChild<QPushButton *>("applyRobotSettings")->click();
    });
    button("setRobotStart")->click();
  }

  void editTiming(double search, double speed = 0.1, double turn = 0.08) {
    QTimer::singleShot(30, this, [=]() {
      auto dialog = window->findChild<QDialog *>("timingDialog");
      if (!dialog) {
        QFAIL("Timing dialog did not open");
      }
      dialog->findChild<QDoubleSpinBox *>("searchCellSeconds")->setValue(search);
      dialog->findChild<QDoubleSpinBox *>("speedCellSeconds")->setValue(speed);
      dialog->findChild<QDoubleSpinBox *>("turn90Seconds")->setValue(turn);
      dialog->grab().save(snapshots + "/robot-speed.png");
      dialog->findChild<QPushButton *>("applyRobotSettings")->click();
    });
    button("setRobotSpeed")->click();
  }

  void beginProbe(const char *name) {
    qputenv("MMS_PROBE_CASE", name);
    button("runAlgorithm")->click();
  }

 private slots:
  void initTestCase() {
    root = QString::fromLocal8Bit(qgetenv("MMS_TEST_ROOT"));
    QVERIFY(!root.isEmpty());
    snapshots = root + "/tools/mms/build/screenshots";
    QVERIFY(QDir().mkpath(snapshots));
    settingsDirectory = new QTemporaryDir(root + "/tools/mms/build/settings-XXXXXX");
    QVERIFY(settingsDirectory->isValid());
    Settings::init();
    QCoreApplication::setOrganizationName("MMRC26-tests");
    QCoreApplication::setApplicationName("native-regressions");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, settingsDirectory->path());
    ColorManager::init();
    auto python = qEnvironmentVariable("MMS_TEST_PYTHON");
    QVERIFY(QFileInfo::exists(python));
    SettingsMouseAlgos::add("probe", root, "", "\"" + python + "\" \"" +
        root + "/tools/mms/tests/protocol_probe.py\"");
    SettingsMouseAlgos::add("solver", root, "", "\"" + python + "\" mms_main.py");
    SettingsMisc::setRecentMouseAlgo("probe");
    SettingsMisc::setRecentMazeFile(":/resources/mazes/blank.num");
    window = new Window();
    window->resize(1280, 800);
    window->show();
    QVERIFY(QTest::qWaitForWindowExposed(window));
    QCOMPARE(Dimensions::tileLength().getMeters(), 0.192);
  }

  void calibrationValidationAndPersistence() {
    MotionTiming timing;
    timing.searchCellSeconds = 0.2;
    timing.speedCellSeconds = 0.1;
    timing.turn90Seconds = 0.08;
    timing.calibrated = true;
    saveMotionTiming(timing);
    QCOMPARE(loadMotionTiming().searchCellSeconds, 0.2);
    QVERIFY(loadMotionTiming().calibrated);
    Settings::get()->update("mmrc26-timing", "search-cell-seconds", "nan");
    QCOMPARE(loadMotionTiming().searchCellSeconds, MotionTiming().searchCellSeconds);
    QVERIFY(!loadMotionTiming().calibrated);
    RobotStart start;
    start.x = 8;
    start.y = 6;
    start.heading = SemiDirection::WEST;
    saveRobotStart("maze-a", start);
    QCOMPARE(loadRobotStart("maze-a", QSize(10, 10)).x, 8);
    QCOMPARE(loadRobotStart("maze-a", QSize(10, 10)).heading, SemiDirection::WEST);
    QCOMPARE(loadRobotStart("maze-a", QSize(4, 4)).x, 0);
    QCOMPARE(loadRobotStart("maze-b", QSize(10, 10)).x, 0);
  }

  void selectedPoseAndRealTimeMotion() {
    editStart(2, 0);
    editTiming(0.2);
    beginProbe("timing");
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("DONE"), 5000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(output().contains("START 2 0 e"));
    QVERIFY(output().contains("WALLS false,true,false,false"));
    QVERIFY(measured("MOVE1") >= 0.19 && measured("MOVE1") < 0.4);
    QVERIFY(measured("MOVE2") >= 0.09 && measured("MOVE2") < 0.3);
    QVERIFY(measured("TURN") >= 0.07 && measured("TURN") < 0.25);
    window->grab().save(snapshots + "/mms-main.png");
    qInfo("Measured search %.3fs, speed %.3fs, turn %.3fs", measured("MOVE1"),
          measured("MOVE2"), measured("TURN"));
  }

  void pauseDoesNotAdvanceMovement() {
    beginProbe("pause");
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("BEFORE_MOVE"), 3000);
    QTest::qWait(40);
    button("pauseAlgorithm")->click();
    QTest::qWait(300);
    QVERIFY(!output().contains("MOVE1"));
    QCOMPARE(button("pauseAlgorithm")->text(), QString("Resume"));
    button("pauseAlgorithm")->click();
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("DONE"), 5000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(measured("MOVE1") >= 0.49 && measured("MOVE1") < 0.75);
  }

  void timingChangesOnlyAffectNextPrimitive() {
    editTiming(0.4);
    beginProbe("snapshot");
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("BEFORE_MOVE"), 3000);
    QTest::qWait(60);
    editTiming(0.1);
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("DONE"), 5000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(measured("MOVE1") >= 0.39 && measured("MOVE1") < 0.6);
    QVERIFY(measured("MOVE2") >= 0.09 && measured("MOVE2") < 0.3);
  }

  void resetKeepsSelectedPose() {
    editTiming(0.2);
    beginProbe("reset");
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("BEFORE_MOVE"), 3000);
    button("resetAlgorithm")->click();
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("DONE"), 5000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(output().contains("RESET true"));
    QVERIFY(output().contains("WALLS false,true,false,false"));
  }

  void changingStartRestartsActiveAlgorithm() {
    editTiming(0.5);
    beginProbe("restart");
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("BEFORE_MOVE"), 3000);
    editStart(3, 0);
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("DONE"), 5000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(output().contains("START 3 0 e"));
    QVERIFY(output().contains("WALLS false,true,false,false"));
    QCOMPARE(loadRobotStart(":/resources/mazes/blank.num", QSize(16, 16)).x, 3);
  }

  void actualSolverWithChangedStart() {
    editTiming(0.01, 0.01, 0.01);
    editStart(15, 15, SemiDirection::SOUTH);
    for (auto combo : window->findChildren<QComboBox *>()) {
      if (combo->findText("solver") >= 0) {
        combo->setCurrentText("solver");
      }
    }
    button("runAlgorithm")->click();
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("CHAMPIONSHIP FINISH!"), 15000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(output().contains("Start: x=15, y=15, heading=SOUTH"));
    QVERIFY(output().contains("Returned to start"));
    QVERIFY(!output().contains("Solver stopped"));
  }

  void actualSolverInCompetitionMaze() {
    QString maze = root + "/mazes_num/mmrc26_10x10/mmrc26_10x10_seed_17.num";
    if (!QFileInfo::exists(maze)) {
      QSKIP("Optional generated competition maze is not installed");
    }
    for (auto combo : window->findChildren<QComboBox *>()) {
      if (combo->currentText().contains(".num")) {
        combo->addItem(maze);
        combo->setCurrentText(maze);
        QMetaObject::invokeMethod(combo, "textActivated", Q_ARG(QString, maze));
      }
    }
    editStart(9, 9, SemiDirection::SOUTH);
    button("runAlgorithm")->click();
    QTRY_VERIFY_WITH_TIMEOUT(output().contains("CHAMPIONSHIP FINISH!"), 20000);
    QTRY_COMPARE(button("runAlgorithm")->text(), QString("Run"));
    QVERIFY(output().contains("Starting MMRC26 Solver in MMS (10x10)"));
    QVERIFY(output().contains("Start: x=9, y=9, heading=SOUTH"));
    QVERIFY(output().contains("Returned to start"));
    window->grab().save(snapshots + "/competition-run.png");
  }

  void cleanupTestCase() {
    window->close();
    delete window;
    delete settingsDirectory;
  }
};

QTEST_MAIN(NativeChecks)
#include "native_tests.moc"
