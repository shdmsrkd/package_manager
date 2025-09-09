#include "../include/package_manager/main_window.hpp"
#include <QTime>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  initializePackageConfig();
  setupConnections();

  ui->statusTextEdit->append("Node Control Program initialized. Ready to control packages.");
}

void MainWindow::initializePackageConfig()
{
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 패키지 괸리 여기서 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (패키지명, launch파일명 또는 노드명)
  packageConfig[1] = QPair<QString, QString>("dynamixel_rdk_ros2", "rdk.launch.py");
  packageConfig[2] = QPair<QString, QString>("ik_walk", "ik_walk");
  packageConfig[3] = QPair<QString, QString>("ebimu_v5", "e2box_imu_9dofv4.launch.py");
  packageConfig[4] = QPair<QString, QString>("gripper_controller", "auto_mode.launch.py");
  packageConfig[5] = QPair<QString, QString>("intelligent_robot_vision", "intelligent_robot_vision.launch.py");
  packageConfig[6] = QPair<QString, QString>("srcirc_master25", "master.launch.py");
  packageConfig[7] = QPair<QString, QString>("tune_walk", "tune_walk");
  packageConfig[8] = QPair<QString, QString>("motion_operator", "motion_operator.launch.py");

  // 패키지 라벨 업데이트
  ui->package1Label->setText(QString("● Package 1: %1").arg(packageConfig[1].first));
  ui->package2Label->setText(QString("● Package 2: %1").arg(packageConfig[2].first));
  ui->package3Label->setText(QString("● Package 3: %1").arg(packageConfig[3].first));
  ui->package4Label->setText(QString("● Package 4: %1").arg(packageConfig[4].first));
  ui->package5Label->setText(QString("● Package 5: %1").arg(packageConfig[5].first));
  ui->package6Label->setText(QString("● Package 6: %1").arg(packageConfig[6].first));
  ui->package7Label->setText(QString("● Package 7: %1").arg(packageConfig[7].first));
  ui->package8Label->setText(QString("● Package 8: %1").arg(packageConfig[8].first));

  // 상태 라벨들을 맵에 연결
  statusLabels[packageConfig[1].first] = ui->package1Label;
  statusLabels[packageConfig[2].first] = ui->package2Label;
  statusLabels[packageConfig[3].first] = ui->package3Label;
  statusLabels[packageConfig[4].first] = ui->package4Label;
  statusLabels[packageConfig[5].first] = ui->package5Label;
  statusLabels[packageConfig[6].first] = ui->package6Label;
  statusLabels[packageConfig[7].first] = ui->package7Label;
  statusLabels[packageConfig[8].first] = ui->package8Label;

  // 초기 스타일 설정 (빨간색 원형)
  QString redStyle = "QLabel { color: red; }";
  ui->package1Label->setStyleSheet(redStyle);
  ui->package2Label->setStyleSheet(redStyle);
  ui->package3Label->setStyleSheet(redStyle);
  ui->package4Label->setStyleSheet(redStyle);
  ui->package5Label->setStyleSheet(redStyle);
  ui->package6Label->setStyleSheet(redStyle);
  ui->package7Label->setStyleSheet(redStyle);
  ui->package8Label->setStyleSheet(redStyle);
}

void MainWindow::setupConnections()
{
  // Qnode 시그널 연결
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(statusMessage(QString)), this, SLOT(onStatusMessage(QString)));
  QObject::connect(qnode, SIGNAL(packageStarted(QString)), this, SLOT(onPackageStarted(QString)));
  QObject::connect(qnode, SIGNAL(packageStopped(QString)), this, SLOT(onPackageStopped(QString)));

  // 패키지 시그널 연결
  // Package 1
  QObject::connect(ui->startPackage1Button, SIGNAL(clicked()), this, SLOT(onStartPackage1()));
  QObject::connect(ui->stopPackage1Button, SIGNAL(clicked()), this, SLOT(onStopPackage1()));
  QObject::connect(ui->restartPackage1Button, SIGNAL(clicked()), this, SLOT(onRestartPackage1()));

  // Package 2
  QObject::connect(ui->startPackage2Button, SIGNAL(clicked()), this, SLOT(onStartPackage2()));
  QObject::connect(ui->stopPackage2Button, SIGNAL(clicked()), this, SLOT(onStopPackage2()));
  QObject::connect(ui->restartPackage2Button, SIGNAL(clicked()), this, SLOT(onRestartPackage2()));

  // Package 3
  QObject::connect(ui->startPackage3Button, SIGNAL(clicked()), this, SLOT(onStartPackage3()));
  QObject::connect(ui->stopPackage3Button, SIGNAL(clicked()), this, SLOT(onStopPackage3()));
  QObject::connect(ui->restartPackage3Button, SIGNAL(clicked()), this, SLOT(onRestartPackage3()));

  // Package 4
  QObject::connect(ui->startPackage4Button, SIGNAL(clicked()), this, SLOT(onStartPackage4()));
  QObject::connect(ui->stopPackage4Button, SIGNAL(clicked()), this, SLOT(onStopPackage4()));
  QObject::connect(ui->restartPackage4Button, SIGNAL(clicked()), this, SLOT(onRestartPackage4()));

  // Package 5
  QObject::connect(ui->startPackage5Button, SIGNAL(clicked()), this, SLOT(onStartPackage5()));
  QObject::connect(ui->stopPackage5Button, SIGNAL(clicked()), this, SLOT(onStopPackage5()));
  QObject::connect(ui->restartPackage5Button, SIGNAL(clicked()), this, SLOT(onRestartPackage5()));

  // Package 6
  QObject::connect(ui->startPackage6Button, SIGNAL(clicked()), this, SLOT(onStartPackage6()));
  QObject::connect(ui->stopPackage6Button, SIGNAL(clicked()), this, SLOT(onStopPackage6()));
  QObject::connect(ui->restartPackage6Button, SIGNAL(clicked()), this, SLOT(onRestartPackage6()));

  // Package 7
  QObject::connect(ui->startPackage7Button, SIGNAL(clicked()), this, SLOT(onStartPackage7()));
  QObject::connect(ui->stopPackage7Button, SIGNAL(clicked()), this, SLOT(onStopPackage7()));
  QObject::connect(ui->restartPackage7Button, SIGNAL(clicked()), this, SLOT(onRestartPackage7()));

  // Package 8
  QObject::connect(ui->startPackage8Button, SIGNAL(clicked()), this, SLOT(onStartPackage8()));
  QObject::connect(ui->stopPackage8Button, SIGNAL(clicked()), this, SLOT(onStopPackage8()));
  QObject::connect(ui->restartPackage8Button, SIGNAL(clicked()), this, SLOT(onRestartPackage8()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::onStatusMessage(const QString& message)
{
  ui->statusTextEdit->append(QString("[%1] %2").arg(QTime::currentTime().toString()).arg(message));
  QTextCursor cursor = ui->statusTextEdit->textCursor();
  cursor.movePosition(QTextCursor::End);
  ui->statusTextEdit->setTextCursor(cursor);
}

// 버튼 활성화 비활성화 처리
void MainWindow::onPackageStarted(const QString& packageName)
{
  updatePackageStatus(packageName, true);
}

void MainWindow::onPackageStopped(const QString& packageName)
{
  updatePackageStatus(packageName, false);
}

void MainWindow::updatePackageStatus(const QString& packageName, bool isRunning)
{
  if (statusLabels.contains(packageName))
  {
    QLabel* packageLabel = statusLabels[packageName];

    // 패키지 번호 찾기
    int packageNum = 0;
    for (int i = 1; i <= 8; i++)
    {
      if (packageConfig[i].first == packageName)
      {
        packageNum = i;
        break;
      }
    }

    if (packageNum > 0)
    {
      if (isRunning)
      {
        // 실행 중: 초록색 체크 표시
        packageLabel->setText(QString("✓ Package %1: %2").arg(packageNum).arg(packageName));
        packageLabel->setStyleSheet("QLabel { color: green; font-weight: bold; }");
      }
      else
      {
        // 정지: 빨간색 원형 표시
        packageLabel->setText(QString("● Package %1: %2").arg(packageNum).arg(packageName));
        packageLabel->setStyleSheet("QLabel { color: red; }");
      }
    }
  }
}


/*====================================== Package 1 ======================================*/
void MainWindow::onStartPackage1()
{
  QString packageName = packageConfig[1].first;
  QString nodeName = packageConfig[1].second;
  qnode->startPackage(packageName, nodeName, LAUNCH);
}

void MainWindow::onStopPackage1()
{
  QString packageName = packageConfig[1].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage1()
{
  QString packageName = packageConfig[1].first;
  QString fileOrNode = packageConfig[1].second;
  qnode->restartPackage(packageName, fileOrNode, LAUNCH);
}

/*====================================== Package 2 ======================================*/
void MainWindow::onStartPackage2()
{
  QString packageName = packageConfig[2].first;
  QString nodeName = packageConfig[2].second;
  qnode->startPackage(packageName, nodeName, RUN);
}

void MainWindow::onStopPackage2()
{
  QString packageName = packageConfig[2].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage2()
{
  QString packageName = packageConfig[2].first;
  QString fileOrNode = packageConfig[2].second;
  qnode->restartPackage(packageName, fileOrNode, RUN);
}

/*====================================== Package 3 ======================================*/
void MainWindow::onStartPackage3()
{
  QString packageName = packageConfig[3].first;
  QString nodeName = packageConfig[3].second;
  qnode->startPackage(packageName, nodeName, RUN);
}

void MainWindow::onStopPackage3()
{
  QString packageName = packageConfig[3].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage3()
{
  QString packageName = packageConfig[3].first;
  QString fileOrNode = packageConfig[3].second;
  qnode->restartPackage(packageName, fileOrNode, RUN);
}

/*====================================== Package 4 ======================================*/
void MainWindow::onStartPackage4()
{
  QString packageName = packageConfig[4].first;
  QString nodeName = packageConfig[4].second;
  qnode->startPackage(packageName, nodeName, LAUNCH);
}

void MainWindow::onStopPackage4()
{
  QString packageName = packageConfig[4].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage4()
{
  QString packageName = packageConfig[4].first;
  QString fileOrNode = packageConfig[4].second;
  qnode->restartPackage(packageName, fileOrNode, LAUNCH);
}

/*====================================== Package 5 ======================================*/
void MainWindow::onStartPackage5()
{
  QString packageName = packageConfig[5].first;
  QString nodeName = packageConfig[5].second;
  qnode->startPackage(packageName, nodeName, LAUNCH);
}

void MainWindow::onStopPackage5()
{
  QString packageName = packageConfig[5].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage5()
{
  QString packageName = packageConfig[5].first;
  QString fileOrNode = packageConfig[5].second;
  qnode->restartPackage(packageName, fileOrNode, LAUNCH);
}

/*====================================== Package 6 ======================================*/
void MainWindow::onStartPackage6()
{
  QString packageName = packageConfig[6].first;
  QString nodeName = packageConfig[6].second;
  qnode->startPackage(packageName, nodeName, LAUNCH);
}

void MainWindow::onStopPackage6()
{
  QString packageName = packageConfig[6].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage6()
{
  QString packageName = packageConfig[6].first;
  QString fileOrNode = packageConfig[6].second;
  qnode->restartPackage(packageName, fileOrNode, LAUNCH);
}

/*====================================== Package 7 ======================================*/
void MainWindow::onStartPackage7()
{
  QString packageName = packageConfig[7].first;
  QString nodeName = packageConfig[7].second;
  qnode->startPackage(packageName, nodeName, RUN);
}

void MainWindow::onStopPackage7()
{
  QString packageName = packageConfig[7].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage7()
{
  QString packageName = packageConfig[7].first;
  QString fileOrNode = packageConfig[7].second;
  qnode->restartPackage(packageName, fileOrNode, RUN);
}

/*====================================== Package 8 ======================================*/
void MainWindow::onStartPackage8()
{
  QString packageName = packageConfig[8].first;
  QString nodeName = packageConfig[8].second;
  qnode->startPackage(packageName, nodeName, RUN);
}

void MainWindow::onStopPackage8()
{
  QString packageName = packageConfig[8].first;
  qnode->stopPackage(packageName);
}

void MainWindow::onRestartPackage8()
{
  QString packageName = packageConfig[8].first;
  QString fileOrNode = packageConfig[8].second;
  qnode->restartPackage(packageName, fileOrNode, RUN);
}
