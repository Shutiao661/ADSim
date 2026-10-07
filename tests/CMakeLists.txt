file(GLOB ADSIM_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)

add_executable(adsim_tests ${ADSIM_TEST_SOURCES} TestMain.cpp)
target_link_libraries(adsim_tests PRIVATE adsim_core adsim_warnings)
target_include_directories(adsim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME adsim_tests COMMAND adsim_tests)
