add_library(server Server.cpp Server.h)
target_link_libraries(server common)

## Instruct CMake to run moc automatically when needed.
##set(CMAKE_AUTOMOC ON)
##set(CMAKE_AUTOUIC ON)
##set(CMAKE_AUTORCC ON)
#
## Find the QtWidgets library
#find_package(Qt5 COMPONENTS Core Widgets Charts REQUIRED)
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#
## Find QWT
#find_path(QWT_INCLUDE_DIR NAMES qwt.h PATHS /usr/local/qwt-6.1.3/include)
#find_library(QWT_LIBRARY NAMES qwt-qt5 qwt PATHS /usr/local/qwt-6.1.3/lib)
#
#add_library(client Client.cpp)
#target_include_directories(client PRIVATE ${QWT_INCLUDE_DIR})
#target_link_libraries(client Qt5::Widgets Qt5::Charts ${QWT_LIBRARY})
add_library(client Client.cpp Client.h)
