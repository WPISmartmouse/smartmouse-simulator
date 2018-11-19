file(GLOB_RECURSE CEREAL_HEADERS *.h)
add_library(cereal ${CEREAL_HEADERS})
add_library(cereal::cereal ALIAS cereal)
target_include_directories(
        cereal
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
set_target_properties(cereal PROPERTIES LINKER_LANGUAGE CXX)

