#include <cstdio>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <thread>

#include <QtWidgets/QApplication>

#include <core/args.h>
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
    ssim::Server server;
    std::thread thread(&ssim::Server::thread_run, &server);

    auto client = std::make_unique<ssim::Client>(&server);
    client->setWindowTitle("Smartmouse Simulator");
    client->showMaximized();
    // Runs until the application is exited
    return_code = QApplication::exec();
    thread.join();
  } while (return_code == ssim::Client::kRestartCode);

  QApplication::quit();

  return return_code;
}
