add_library(test_robot test_plugin.cpp test_description.cpp)
target_link_libraries(test_robot hal core)

