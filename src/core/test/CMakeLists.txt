file(GLOB TEST_SRCS *.cpp)

foreach (TEST_SRC ${TEST_SRCS})
    get_filename_component(TARGET ${TEST_SRC} NAME_WE)
    add_executable(${TARGET} ${TEST_SRC})
    target_link_libraries(${TARGET} core gtest gtest_main)
    add_test(NAME ${TARGET} COMMAND ${TARGET})
    sanitize(${TARGET})
endforeach ()
