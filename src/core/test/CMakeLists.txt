file(GLOB TEST_SRCS *_TEST.cpp)

add_library(mock mock_mouse.cpp)
target_include_directories(mock PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(mock core)

foreach (TEST_SRC ${TEST_SRCS})
    get_filename_component(TARGET ${TEST_SRC} NAME_WE)
    add_executable(${TARGET} ${TEST_SRC})
    target_link_libraries(${TARGET} core mock gtest gtest_main gcov)
    gtest_discover_tests(${TARGET})
    sanitize(${TARGET})
endforeach ()
