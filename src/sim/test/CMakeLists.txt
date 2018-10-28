file(GLOB TEST_SRCS *_TEST.cpp)

foreach (TEST_SRC ${TEST_SRCS})
    get_filename_component(TARGET ${TEST_SRC} NAME_WE)
    add_executable(${TARGET} ${TEST_SRC})
    target_link_libraries(${TARGET} sim core mock gtest gtest_main gcov)
    gtest_discover_tests(${TARGET})
    sanitize(${TARGET})
endforeach ()
