#include <core/args.h>
#include <cstdio>
#include <memory>
#include <cstdlib>

#include <QtWidgets/QApplication>

#include <sim/server.h>
#include <sim/client.h>

int main(int argc, char *argv[]) {
  int return_code = 0;

  qRegisterMetaType<ssim::RobotCommand>("RobotCommand");
  qRegisterMetaType<ssim::PhysicsConfig>("PhysicsConfig");
  qRegisterMetaType<ssim::ServerControl>("ServerControl");
  qRegisterMetaType<ssim::AbstractMaze>("AbstractMaze");
  qRegisterMetaType<ssim::RobotDescription>("RobotDescription");
  qRegisterMetaType<ssim::RobotSimState>("RobotSimState");
  qRegisterMetaType<ssim::WorldStatistics>("WorldStatistics");

  do {
    QApplication app(argc, argv);

    // Start physics thread
    QThread thread;
    ssim::Server server;
    server.moveToThread(&thread);
    QObject::connect(&thread, SIGNAL(started()), &server, SLOT(process()));
    QObject::connect(&server, SIGNAL(finished()), &thread, SLOT(quit()));
    QObject::connect(&server, SIGNAL(finished()), &server, SLOT(deleteLater()));
    QObject::connect(&thread, SIGNAL(finished()), &thread, SLOT(deleteLater()));
    thread.start();

    auto client = std::make_unique<ssim::Client>(&server);
    client->setWindowTitle("Smartmouse Simulator");
    client->showMaximized();
    // Runs until the application is exited
    return_code = QApplication::exec();
  } while (return_code == ssim::Client::kRestartCode);

  QApplication::quit();

  return return_code;
}
