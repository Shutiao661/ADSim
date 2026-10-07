file(GLOB ADSIM_CLI_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(adsim_cli ${ADSIM_CLI_SOURCES})
target_link_libraries(adsim_cli PRIVATE adsim_core adsim_warnings)
