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
  std::unique_ptr<ssim::Client> window;

  do {
    // Start physics thread
    ssim::Server server;
    server.Start();

    QApplication app(argc, argv);
    window = std::make_unique<ssim::Client>();
    window->setWindowTitle("Smartmouse Simulator");
    window->showMaximized();
    // Runs until the application is exited
    return_code = QApplication::exec();

    // We need to restart the server
    ssim::ServerControl quit_msg;
    quit_msg.quit = true;
    server.OnServerControl(quit_msg);
    server.Join();
  } while (return_code == ssim::Client::kRestartCode);

  QApplication::quit();

  return return_code;
}
