#ifndef package_manager_QNODE_HPP_
#define package_manager_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#endif
#include <QThread>
#include <QString>
#include <QProcess>
#include <map>

struct PackageInfo {
  std::string name;
  std::string executable;
  std::string type;
};

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  #define LAUNCH 0
  #define RUN 1

  void startPackage(const QString& packageName, const QString& fileOrNodeName, int type);
  void stopPackage(const QString& packageName);
  void restartPackage(const QString& packageName, const QString& fileOrNodeName, int type);
  void initParameters();

  // 패키지 정보 가져오는 함수들
  QString getPackageName(int packageIndex);
  QString getPackageExecutable(int packageIndex);
  QString getPackageType(int packageIndex);
  void startPackageByIndex(int packageIndex);

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  std::map<QString, QProcess*> packageProcesses;

  // Package parameters
  std::string package1_name, package1_executable, package1_type;
  std::string package2_name, package2_executable, package2_type;
  std::string package3_name, package3_executable, package3_type;
  std::string package4_name, package4_executable, package4_type;
  std::string package5_name, package5_executable, package5_type;
  std::string package6_name, package6_executable, package6_type;
  std::string package7_name, package7_executable, package7_type;
  std::string package8_name, package8_executable, package8_type;

  std::map<int, PackageInfo> packages;

  void killPackageNodes(const QString& packageName);

Q_SIGNALS:
  void rosShutDown();
  void statusMessage(const QString& message);
  void packageStarted(const QString& packageName);
  void packageStopped(const QString& packageName);
};

#endif /* package_manager_QNODE_HPP_ */
