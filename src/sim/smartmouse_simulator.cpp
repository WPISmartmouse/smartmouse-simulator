#include <core/args.h>
#include <cstdio>
#include <memory>
#include <cstdlib>

#include <QtWidgets/QApplication>

#include <sim/server.h>
#include <sim/client.h>

void PrintVersionInfo() {
  printf("smartmouse_simulator v1.0.0\n");
}

int main(int argc, char *argv[]) {
  int return_code = 0;
  std::unique_ptr<ssim::Client> client;

  do {
    // Start physics thread
    ssim::Server server;

    QApplication app(argc, argv);
    client = std::make_unique<ssim::Client>(server);
    client->setWindowTitle("Smartmouse Simulator");
    client->showMaximized();
    // Runs until the application is exited
    return_code = QApplication::exec();
  } while (return_code == ssim::Client::kRestartCode);

  QApplication::quit();

  return return_code;
}
