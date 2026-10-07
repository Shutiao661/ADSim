cmake_minimum_required(VERSION 3.16)

# =============================================================================
#  ADSim — 基于 C++ 与 CARLA 的自动驾驶行为决策仿真及数据分析平台
#
#  设计原则：
#    * adsim_core 为纯 C++17 核心库，零外部依赖，任何环境下均可编译运行。
#    * CARLA / Ceres / Qt / ROS / libcurl 等重型依赖通过开关按需接入，
#      默认关闭——对应源文件仍在工程内，按真实 API 编写，接入即用。
# =============================================================================

project(ADSim
        VERSION 1.0.0
        DESCRIPTION "Autonomous Driving behavior-decision Simulation & data-analysis platform"
        LANGUAGES CXX)

# ---------------------------------------------------------------------------
# 构建选项
# ---------------------------------------------------------------------------
option(ADSIM_BUILD_TESTS   "构建单元测试"              ON)
option(ADSIM_BUILD_TOOLS   "构建工具程序(数据生成等)"    ON)
option(ADSIM_BUILD_BENCH   "构建性能基准测试"           ON)
option(ADSIM_WITH_CARLA    "接入 CARLA 仿真引擎"        OFF)
option(ADSIM_WITH_CERES    "接入 Ceres Solver 数值优化" OFF)
option(ADSIM_WITH_QT       "接入 Qt 可视化前端"         OFF)
option(ADSIM_WITH_ROS      "接入 ROS 生态"              OFF)
option(ADSIM_WITH_CURL     "接入 libcurl (LLM HTTP)"    OFF)
option(ADSIM_WITH_COMPRESSION "启用 bag 压缩块解压 (bz2/lz4，自动探测)" ON)

# CARLA 根目录，仅在 ADSIM_WITH_CARLA=ON 时使用
set(ADSIM_CARLA_ROOT "/opt/carla" CACHE PATH "CARLA 安装根目录")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

# ---------------------------------------------------------------------------
# 编译告警
# ---------------------------------------------------------------------------
add_library(adsim_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(adsim_warnings INTERFACE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
    -Wshadow
    -Wnon-virtual-dtor)
endif()

# ---------------------------------------------------------------------------
# 核心库
# ---------------------------------------------------------------------------
file(GLOB_RECURSE ADSIM_CORE_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# 依赖外部重型框架的源文件按开关剔除，默认不参与构建
set(ADSIM_OPTIONAL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sim/carla_bridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planning/optimizer/ceres_solver_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/viz/qt_visualizer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/agent/http_llm_client.cpp)

foreach(_opt_src ${ADSIM_OPTIONAL_SOURCES})
  list(REMOVE_ITEM ADSIM_CORE_SOURCES ${_opt_src})
endforeach()

add_library(adsim_core STATIC ${ADSIM_CORE_SOURCES})
target_include_directories(adsim_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adsim_core PRIVATE adsim_warnings)

find_package(Threads REQUIRED)
target_link_libraries(adsim_core PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------
# 压缩块解压（bz2 / lz4）
#
# 与上面几个开关不同，这里是**自动探测**：系统上有库就启用，没有就编译为
# 不支持并在运行时给出明确提示。理由是实车 bag 经常是压缩存储的，
# 不支持的解析器实际可用性有限；但把 bz2/lz4 变成硬依赖又会破坏
# "核心库零依赖可构建"的承诺。自动探测同时满足这两点。
# ---------------------------------------------------------------------------
if(ADSIM_WITH_COMPRESSION)
  find_package(BZip2 QUIET)
  if(BZip2_FOUND)
    target_link_libraries(adsim_core PRIVATE BZip2::BZip2)
    target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_BZ2=1)
    message(STATUS "ADSIM: bzip2 已启用 (${BZIP2_VERSION_STRING})")
  else()
    message(STATUS "ADSIM: 未找到 bzip2 —— bz2 压缩块将不可读")
  endif()

  # CMake 没有内置的 FindLZ4，手动查找
  find_path(ADSIM_LZ4_INCLUDE_DIR NAMES lz4frame.h)
  find_library(ADSIM_LZ4_LIBRARY NAMES lz4)
  if(ADSIM_LZ4_INCLUDE_DIR AND ADSIM_LZ4_LIBRARY)
    target_include_directories(adsim_core PRIVATE ${ADSIM_LZ4_INCLUDE_DIR})
    target_link_libraries(adsim_core PRIVATE ${ADSIM_LZ4_LIBRARY})
    target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_LZ4=1)
    message(STATUS "ADSIM: LZ4 已启用 (${ADSIM_LZ4_LIBRARY})")
  else()
    message(STATUS "ADSIM: 未找到 LZ4 —— lz4 压缩块将不可读")
  endif()
endif()

# ---------------------------------------------------------------------------
# 可选依赖：CARLA
# ---------------------------------------------------------------------------
if(ADSIM_WITH_CARLA)
  if(NOT EXISTS ${ADSIM_CARLA_ROOT})
    message(FATAL_ERROR "ADSIM_WITH_CARLA=ON 但 ADSIM_CARLA_ROOT=${ADSIM_CARLA_ROOT} 不存在")
  endif()
  target_sources(adsim_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/sim/carla_bridge.cpp)
  target_include_directories(adsim_core PRIVATE
      ${ADSIM_CARLA_ROOT}/include
      ${ADSIM_CARLA_ROOT}/LibCarla/source
      ${ADSIM_CARLA_ROOT}/carla)
  target_link_directories(adsim_core PRIVATE ${ADSIM_CARLA_ROOT}/lib)
  target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_CARLA=1)
  message(STATUS "ADSIM: CARLA 已启用 (${ADSIM_CARLA_ROOT})")
endif()

# ---------------------------------------------------------------------------
# 可选依赖：Ceres Solver
# ---------------------------------------------------------------------------
if(ADSIM_WITH_CERES)
  find_package(Ceres QUIET)
  if(NOT Ceres_FOUND)
    message(FATAL_ERROR
      "ADSIM_WITH_CERES=ON 但未找到 Ceres Solver。\n"
      "  方式一（推荐）：sudo apt install libceres-dev\n"
      "  方式二（无 root 权限时源码安装到自定义前缀，再用 CMAKE_PREFIX_PATH 指过去）：\n"
      "      cmake -S . -B build -DADSIM_WITH_CERES=ON -DCMAKE_PREFIX_PATH=<ceres安装前缀>\n"
      "  提示：路径优化在后端不可用时并不受影响，去掉该开关即回退到内置 LM 求解器。")
  endif()
  target_sources(adsim_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/planning/optimizer/ceres_solver_backend.cpp)
  target_link_libraries(adsim_core PRIVATE Ceres::ceres)
  target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_CERES=1)
  message(STATUS "ADSIM: Ceres Solver 已启用 (${CERES_VERSION})")
endif()

# ---------------------------------------------------------------------------
# 可选依赖：Qt
# ---------------------------------------------------------------------------
if(ADSIM_WITH_QT)
  find_package(Qt5 COMPONENTS Widgets QUIET)
  if(NOT Qt5_FOUND)
    find_package(Qt6 COMPONENTS Widgets QUIET)
  endif()
  if(NOT Qt5_FOUND AND NOT Qt6_FOUND)
    message(FATAL_ERROR "ADSIM_WITH_QT=ON 但未找到 Qt5/Qt6 Widgets")
  endif()
  target_sources(adsim_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/viz/qt_visualizer.cpp)
  if(Qt5_FOUND)
    target_link_libraries(adsim_core PRIVATE Qt5::Widgets)
  else()
    target_link_libraries(adsim_core PRIVATE Qt6::Widgets)
  endif()
  target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_QT=1)
  message(STATUS "ADSIM: Qt 可视化已启用")
endif()

# ---------------------------------------------------------------------------
# 可选依赖：ROS
# ---------------------------------------------------------------------------
if(ADSIM_WITH_ROS)
  find_package(catkin REQUIRED COMPONENTS roscpp rosbag sensor_msgs nav_msgs)
  target_link_libraries(adsim_core PRIVATE ${catkin_LIBRARIES})
  target_include_directories(adsim_core PRIVATE ${catkin_INCLUDE_DIRS})
  target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_ROS=1)
  message(STATUS "ADSIM: ROS 已启用")
endif()

# ---------------------------------------------------------------------------
# 可选依赖：libcurl
# ---------------------------------------------------------------------------
if(ADSIM_WITH_CURL)
  find_package(CURL REQUIRED)
  target_sources(adsim_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/agent/http_llm_client.cpp)
  target_link_libraries(adsim_core PRIVATE CURL::libcurl)
  target_compile_definitions(adsim_core PUBLIC ADSIM_HAS_CURL=1)
  message(STATUS "ADSIM: libcurl 已启用")
endif()

# ---------------------------------------------------------------------------
# 子目录
# ---------------------------------------------------------------------------
add_subdirectory(apps)

if(ADSIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(ADSIM_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

# ---------------------------------------------------------------------------
# 构建摘要
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "==================== ADSim 配置摘要 ====================")
message(STATUS "  版本          : ${PROJECT_VERSION}")
message(STATUS "  构建类型      : ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ 标准      : ${CMAKE_CXX_STANDARD}")
message(STATUS "  编译器        : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  压缩块解压    : ${ADSIM_WITH_COMPRESSION}")
message(STATUS "  CARLA         : ${ADSIM_WITH_CARLA}")
message(STATUS "  Ceres         : ${ADSIM_WITH_CERES}")
message(STATUS "  Qt            : ${ADSIM_WITH_QT}")
message(STATUS "  ROS           : ${ADSIM_WITH_ROS}")
message(STATUS "  libcurl       : ${ADSIM_WITH_CURL}")
message(STATUS "  单元测试      : ${ADSIM_BUILD_TESTS}")
message(STATUS "=======================================================")
message(STATUS "")
