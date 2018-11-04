## Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets Charts REQUIRED)

# Find QWT
find_path(QWT_INCLUDE_DIR NAMES qwt.h PATHS /usr/include/qwt)
find_library(QWT_LIBRARY NAMES qwt-qt5 PATHS /usr/lib/)

add_library(sim ray_tracing.cpp)
target_include_directories(sim PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(sim core Eigen3::Eigen)

add_library(server server.cpp)
target_include_directories(server PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(server sim core hal kinematic_controller Qt5::Core)

file(GLOB WIDGET_SRCS widgets/*.cpp)

add_library(client client.cpp ${WIDGET_SRCS})
target_include_directories(client PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${QWT_INCLUDE_DIR})
target_link_libraries(client sim core hal Qt5::Widgets ${QWT_LIBRARY} dl)

add_library(smartmouse_simulator smartmouse_simulator.cpp simulator.qrc)
target_include_directories(smartmouse_simulator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(smartmouse_simulator client server core Threads::Threads)

file(GLOB WIDGET_SRCS widgets/*.cpp)

add_subdirectory(test)
