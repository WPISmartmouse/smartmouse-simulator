file(GLOB SRCS *.cpp)
file(GLOB HDRS *.h)

add_library(kinematic_controller ${SRCS} ${HDRS})
target_include_directories(kinematic_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kinematic_controller core Eigen3::Eigen)
