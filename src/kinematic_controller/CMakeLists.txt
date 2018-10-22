file(GLOB SRCS *.cpp)

add_library(kinematic_controller ${SRCS})
target_include_directories(kinematic_controller PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(kinematic_controller core hal Eigen3::Eigen)
