## Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Find QWT
#find_path(QWT_INCLUDE_DIR NAMES qwt.h PATHS /usr/local/qwt-6.1.3/include)
#find_library(QWT_LIBRARY NAMES qwt-qt5 qwt PATHS /usr/local/qwt-6.1.3/lib)

add_library(sim ray_tracing.cpp time.cpp conversions.cpp)
target_include_directories(sim PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(sim core Eigen3::Eigen)

add_library(server server.cpp)
target_include_directories(server PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(server sim core teensyduino kinematic_controller Threads::Threads)

add_library(client client.cpp include/sim/client.h)
target_include_directories(client PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${QWT_INCLUDE_DIR})
target_link_libraries(client sim core teensyduino Qt5::Widgets ${QWT_LIBRARY} dl)

add_executable(smartmouse_simulator smartmouse_simulator.cpp ${WIDGET_SRCS} simulator.qrc)
target_include_directories(smartmouse_simulator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(smartmouse_simulator client server core)
