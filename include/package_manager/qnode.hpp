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

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  std::map<QString, QProcess*> packageProcesses;

  void killPackageNodes(const QString& packageName);

Q_SIGNALS:
  void rosShutDown();
  void statusMessage(const QString& message);
  void packageStarted(const QString& packageName);
  void packageStopped(const QString& packageName);
};

#endif /* package_manager_QNODE_HPP_ */
