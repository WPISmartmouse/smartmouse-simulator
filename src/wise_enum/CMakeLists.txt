file(GLOB_RECURSE HEADERS *.h)
add_library(wise_enum ${HEADERS})
add_library(wise_enum::wise_enum ALIAS wise_enum)
target_include_directories(
        wise_enum
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
set_target_properties(wise_enum PROPERTIES LINKER_LANGUAGE CXX)

