# =============================================================================
#  adsim_viz — Qt 交互式仿真复盘前端
#
#  仅在 ADSIM_WITH_QT=ON 时构建（父级 apps/CMakeLists.txt 已做条件判断）。
#  Qt 的查找与链接在根 CMakeLists 中完成，这里只负责组装可执行文件。
# =============================================================================

file(GLOB ADSIM_VIZ_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(adsim_viz ${ADSIM_VIZ_SOURCES})
target_link_libraries(adsim_viz PRIVATE adsim_core adsim_warnings)

# Qt 是 GUI 程序：Windows 下不弹控制台，macOS 下打成 app bundle
set_target_properties(adsim_viz PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE)
