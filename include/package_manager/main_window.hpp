#ifndef package_manager_MAIN_WINDOW_H
#define package_manager_MAIN_WINDOW_H

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <QMap>
#include <QPair>


class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

private slots:
  void onStatusMessage(const QString& message);
  void onPackageStarted(const QString& packageName);
  void onPackageStopped(const QString& packageName);

  // 버튼별 슬롯함수
  void onStartPackage1(); void onStopPackage1(); void onRestartPackage1();
  void onStartPackage2(); void onStopPackage2(); void onRestartPackage2();
  void onStartPackage3(); void onStopPackage3(); void onRestartPackage3();
  void onStartPackage4(); void onStopPackage4(); void onRestartPackage4();
  void onStartPackage5(); void onStopPackage5(); void onRestartPackage5();
  void onStartPackage6(); void onStopPackage6(); void onRestartPackage6();
  void onStartPackage7(); void onStopPackage7(); void onRestartPackage7();
  void onStartPackage8(); void onStopPackage8(); void onRestartPackage8();

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
  void setupConnections();
  void initializePackageConfig();
  void updatePackageStatus(const QString& packageName, bool isRunning);

  QMap<int, QPair<QString, QString>> packageConfig;
  QMap<QString, QLabel*> statusLabels;  // 패키지별 상태 표시 라벨
};

#endif  // package_manager_MAIN_WINDOW_H
