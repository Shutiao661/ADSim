file(GLOB ADSIM_TOOL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(_tool_src ${ADSIM_TOOL_SOURCES})
  get_filename_component(_tool_name ${_tool_src} NAME_WE)
  add_executable(${_tool_name} ${_tool_src})
  target_link_libraries(${_tool_name} PRIVATE adsim_core adsim_warnings)
endforeach()
