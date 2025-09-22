#include "../include/package_manager/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("package_manager_node");
  initParameters();
  this->start();
}

QNode::~QNode()
{
  // 소멸자 : 모든 실행 패키지 강제 종료, present:패키지 이름, second: process 객체
  for (auto present = packageProcesses.begin(); present != packageProcesses.end(); present++)
  {
    // nullptr 체크 및 실행 중인지 확인
    if (present->second && present->second->state() == QProcess::Running)
    {
      present->second->kill();
      present->second->waitForFinished(3000);
    }
    delete present->second;
  }
  packageProcesses.clear();

  if (rclcpp::ok()) { rclcpp::shutdown(); }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::initParameters()
{
  node->declare_parameter("packages.package1.name", "dynamixel_rdk_ros2");
  node->declare_parameter("packages.package1.executable", "rdk.launch.py");
  node->declare_parameter("packages.package1.type", "launch");

  node->declare_parameter("packages.package2.name", "ik_walk");
  node->declare_parameter("packages.package2.executable", "ik_walk");
  node->declare_parameter("packages.package2.type", "run");

  node->declare_parameter("packages.package3.name", "ebimu_v5");
  node->declare_parameter("packages.package3.executable", "e2box_imu_9dofv4.launch.py");
  node->declare_parameter("packages.package3.type", "launch");

  node->declare_parameter("packages.package4.name", "gripper_controller");
  node->declare_parameter("packages.package4.executable", "auto_mode.launch.py");
  node->declare_parameter("packages.package4.type", "launch");

  node->declare_parameter("packages.package5.name", "intelligent_robot_vision");
  node->declare_parameter("packages.package5.executable", "intelligent_robot_vision.launch.py");
  node->declare_parameter("packages.package5.type", "launch");

  node->declare_parameter("packages.package6.name", "srcirc_master25");
  node->declare_parameter("packages.package6.executable", "master.launch.py");
  node->declare_parameter("packages.package6.type", "launch");

  node->declare_parameter("packages.package7.name", "tune_walk");
  node->declare_parameter("packages.package7.executable", "tune_walk");
  node->declare_parameter("packages.package7.type", "run");

  node->declare_parameter("packages.package8.name", "motion_operator");
  node->declare_parameter("packages.package8.executable", "motion_operator.launch.py");
  node->declare_parameter("packages.package8.type", "launch");

  node->get_parameter("packages.package1.name", package1_name);
  node->get_parameter("packages.package1.executable", package1_executable);
  node->get_parameter("packages.package1.type", package1_type);

  node->get_parameter("packages.package2.name", package2_name);
  node->get_parameter("packages.package2.executable", package2_executable);
  node->get_parameter("packages.package2.type", package2_type);

  node->get_parameter("packages.package3.name", package3_name);
  node->get_parameter("packages.package3.executable", package3_executable);
  node->get_parameter("packages.package3.type", package3_type);

  node->get_parameter("packages.package4.name", package4_name);
  node->get_parameter("packages.package4.executable", package4_executable);
  node->get_parameter("packages.package4.type", package4_type);

  node->get_parameter("packages.package5.name", package5_name);
  node->get_parameter("packages.package5.executable", package5_executable);
  node->get_parameter("packages.package5.type", package5_type);

  node->get_parameter("packages.package6.name", package6_name);
  node->get_parameter("packages.package6.executable", package6_executable);
  node->get_parameter("packages.package6.type", package6_type);

  node->get_parameter("packages.package7.name", package7_name);
  node->get_parameter("packages.package7.executable", package7_executable);
  node->get_parameter("packages.package7.type", package7_type);

  node->get_parameter("packages.package8.name", package8_name);
  node->get_parameter("packages.package8.executable", package8_executable);
  node->get_parameter("packages.package8.type", package8_type);

  packages[1] = {package1_name, package1_executable, package1_type};
  packages[2] = {package2_name, package2_executable, package2_type};
  packages[3] = {package3_name, package3_executable, package3_type};
  packages[4] = {package4_name, package4_executable, package4_type};
  packages[5] = {package5_name, package5_executable, package5_type};
  packages[6] = {package6_name, package6_executable, package6_type};
  packages[7] = {package7_name, package7_executable, package7_type};
  packages[8] = {package8_name, package8_executable, package8_type};
}

void QNode::startPackage(const QString& packageName, const QString& fileOrNodeName, int type)
{
  if (packageName.isEmpty())
  {
    Q_EMIT statusMessage("Package name cannot be empty!");
    return;
  }

  // 이미 실행 중인지 확인(키 찾기)
  bool isAlreadyRunning = packageProcesses.find(packageName) != packageProcesses.end() && packageProcesses[packageName]->state() == QProcess::Running;
  if (isAlreadyRunning)
  {
    Q_EMIT statusMessage(QString("Package '%1' is already running!").arg(packageName));
    return;
  }

  // 프로세스 객체 생성
  QProcess* process = new QProcess();

  // GUI 설정
  // QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  // env.insert("DISPLAY", ":0");
  // env.insert("QT_X11_NO_MITSHM", "1");
  // process->setProcessEnvironment(env);

  // (ros2 launch) or (ros2 run)
  QString command = "ros2";
  QStringList arguments;

  if (type == LAUNCH) // LAUNCH
  {
    arguments << "launch" << packageName;
    if (!fileOrNodeName.isEmpty()) { arguments << fileOrNodeName; }
  }
  else // RUN
  {
    arguments << "run" << packageName;
    if (!fileOrNodeName.isEmpty()) { arguments << fileOrNodeName; }
    else { arguments << packageName; }
  }

  // 프로세스 종료시 시그널 연결
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, packageName](int exitCode, QProcess::ExitStatus exitStatus)
          {
            QString statusText;
            if (exitCode == 0 && exitStatus == QProcess::NormalExit) { statusText = QString("Package '%1' stopped successfully").arg(packageName); }
            else if (exitStatus == QProcess::CrashExit) { statusText = QString("Package '%1' crashed (exit code: %2)").arg(packageName).arg(exitCode); }
            else { statusText = QString("Package '%1' stopped with error (exit code: %2)").arg(packageName).arg(exitCode); }
            Q_EMIT statusMessage(statusText);
            Q_EMIT packageStopped(packageName);
          });

  // 프로세스 에러 시그널 연결
  connect(process, &QProcess::errorOccurred, [this, packageName](QProcess::ProcessError error)
  {
    QString errorText;
    switch(error) {
      case QProcess::FailedToStart:
        errorText = QString("Package '%1' failed to start - command not found or insufficient permissions").arg(packageName);
        break;
      case QProcess::Crashed:
        errorText = QString("Package '%1' crashed").arg(packageName);
        break;
      case QProcess::Timedout:
        errorText = QString("Package '%1' timed out").arg(packageName);
        break;
      default:
        errorText = QString("Package '%1' encountered an unknown error").arg(packageName);
        break;
    }
    Q_EMIT statusMessage(errorText);
    Q_EMIT packageStopped(packageName);
  });

  // 프로세스 시작시 시그널 연결
  connect(process, &QProcess::started, [this, packageName]()
  {
    Q_EMIT statusMessage(QString("Package '%1' started successfully!").arg(packageName));
    Q_EMIT packageStarted(packageName);
  });

  // 명령어 실행
  process->start(command, arguments);
  packageProcesses[packageName] = process;

  Q_EMIT statusMessage(QString("Starting package '%1'...").arg(packageName));
}

void QNode::stopPackage(const QString& packageName)
{
  if (packageName.isEmpty())
  { Q_EMIT statusMessage("Package name is empty!!!"); return; }

  if (packageProcesses.find(packageName) != packageProcesses.end())
  {
    QProcess* process = packageProcesses[packageName];
    if (process && process->state() == QProcess::Running)
    {
      process->terminate();
      if (!process->waitForFinished(5000))
      {
        process->kill();
        process->waitForFinished(3000);
      }
      Q_EMIT statusMessage(QString("Package '%1' stopped").arg(packageName));
      Q_EMIT packageStopped(packageName);
    }
    else
    {
      Q_EMIT statusMessage(QString("Package '%1' is not running").arg(packageName));
    }
  }

  else
  {
    killPackageNodes(packageName);
    Q_EMIT statusMessage(QString("Attempted to stop package '%1' nodes").arg(packageName));
  }
}

void QNode::restartPackage(const QString& packageName, const QString& fileOrNodeName, int type)
{
  Q_EMIT statusMessage(QString("Restarting package '%1'...").arg(packageName));
  stopPackage(packageName);
  QThread::msleep(800);
  startPackage(packageName, fileOrNodeName, type);
}

void QNode::killPackageNodes(const QString& packageName)
{
  QString killCommand = QString("pkill -f '%1'").arg(packageName);
  QProcess process;
  process.start("bash", QStringList() << "-c" << killCommand);
  process.waitForFinished();
}

QString QNode::getPackageName(int packageIndex)
{
  if (packages.find(packageIndex) != packages.end()) {
    return QString::fromStdString(packages[packageIndex].name);
  }
  return "";
}

QString QNode::getPackageExecutable(int packageIndex)
{
  if (packages.find(packageIndex) != packages.end()) {
    return QString::fromStdString(packages[packageIndex].executable);
  }
  return "";
}

QString QNode::getPackageType(int packageIndex)
{
  if (packages.find(packageIndex) != packages.end()) {
    return QString::fromStdString(packages[packageIndex].type);
  }
  return "";
}

void QNode::startPackageByIndex(int packageIndex)
{
  QString packageName = getPackageName(packageIndex);
  QString executable = getPackageExecutable(packageIndex);
  QString type = getPackageType(packageIndex);

  if (packageName.isEmpty()) {
    Q_EMIT statusMessage(QString("Package %1 not found!").arg(packageIndex));
    return;
  }

  int typeValue = (type == "launch") ? LAUNCH : RUN;

  startPackage(packageName, executable, typeValue);
}
