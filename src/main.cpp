#include <QApplication>
#include <iostream>

#include "../include/package_manager/main_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
