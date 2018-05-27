file(GLOB CORE_SRCS
        AbstractMaze.cpp
        Direction.cpp
        Flood.cpp
        mouse.cpp
        Node.cpp
        Pose.cpp
        SensorReading.cpp
        Solver.cpp
        WallFollow.cpp)
file(GLOB CORE_HDRS
        AbstractMaze.h
        Direction.h
        Flood.h
        math_util.h
        mouse.h
        Node.h
        Plugin.h
        Pose.h
        SensorReading.h
        Solver.h
        WallFollow.h)

add_library(core ${CORE_SRCS} ${CORE_HDRS})
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (${BUILD_TESTS})
    file(GLOB TEST_SRCS *_TEST.cpp)
    foreach (TEST_SRC ${TEST_SRCS})
        get_filename_component(TARGET ${TEST_SRC} NAME_WE)
        add_executable(${TARGET} ${TEST_SRC})
        target_link_libraries(${TARGET} core gtest gtest_main)
        gtest_discover_tests(${TARGET})
    endforeach ()
endif ()
