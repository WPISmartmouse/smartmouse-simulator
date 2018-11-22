project(core VERSION 0.1)

file(GLOB CORE_SRCS *.cpp)

add_library(core ${CORE_SRCS})
target_include_directories(
  core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(core PUBLIC wise_enum::wise_enum cereal::cereal fmt::fmt Eigen3::Eigen)

if(${BUILD_TESTS})
  add_subdirectory(test)
endif()
