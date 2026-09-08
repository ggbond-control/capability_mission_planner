cmake_minimum_required(VERSION 3.16)
project(capability_mission_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTING "Build tests" ON)
option(BUILD_IMAGE_MAP_DEMO "Build the PNG map validation example" ON)
option(BUILD_OFFLINE_MAP_ADAPTER "Build the offline Nav2 map adapter" ON)
option(BUILD_SCENARIO_TESTS "Run tests requiring capability_mission_scenarios" OFF)
set(SCENARIO_ROOT "" CACHE PATH "Path to capability_mission_scenarios")

add_library(capability_mission_planner
  src/grid_path_planner.cpp
  src/mission_planner.cpp
  src/cbs_coordinator.cpp
  src/task_metadata.cpp)

target_include_directories(capability_mission_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_options(capability_mission_planner PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mission_demo examples/mission_demo.cpp)
target_link_libraries(mission_demo PRIVATE capability_mission_planner)

if(BUILD_OFFLINE_MAP_ADAPTER)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
  find_package(yaml-cpp REQUIRED)
  add_library(capability_mission_planner_map_adapter
    src/offline_map_planner/map_bundle_loader.cpp
    src/offline_map_planner/multi_map_path_planner.cpp
    src/offline_map_planner/offline_mission_planner.cpp
    src/offline_map_planner/offline_planner_config.cpp
    src/offline_map_planner/multi_map_coordinator.cpp
    src/offline_map_planner/plan_exporter.cpp
    src/offline_map_planner/live_reservation.cpp)
  target_include_directories(capability_mission_planner_map_adapter
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(capability_mission_planner_map_adapter
    PUBLIC capability_mission_planner
    PRIVATE yaml-cpp ${OpenCV_LIBS})
  target_compile_options(capability_mission_planner_map_adapter
    PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(offline_map_demo examples/offline_map_demo.cpp)
  target_link_libraries(offline_map_demo PRIVATE capability_mission_planner_map_adapter)

  add_executable(capability_mission_planner_cli examples/offline_planner_cli.cpp)
  target_link_libraries(capability_mission_planner_cli PRIVATE
    capability_mission_planner_map_adapter)
endif()

if(BUILD_IMAGE_MAP_DEMO)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
  add_executable(image_map_demo examples/image_map_demo.cpp)
  target_include_directories(image_map_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(image_map_demo PRIVATE
    capability_mission_planner ${OpenCV_LIBS})
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(mission_tests test/mission_tests.cpp)
  target_link_libraries(mission_tests PRIVATE capability_mission_planner)
  add_test(NAME mission_tests COMMAND mission_tests)
  if(TARGET image_map_demo)
    add_test(NAME image_map_validation COMMAND image_map_demo
      ${CMAKE_CURRENT_SOURCE_DIR}/examples/data/image.png
      ${CMAKE_CURRENT_BINARY_DIR}/capability_mission_plan.png)
  endif()
  if(TARGET capability_mission_planner_map_adapter)
    add_executable(live_reservation_tests test/live_reservation_tests.cpp)
    target_link_libraries(live_reservation_tests PRIVATE capability_mission_planner_map_adapter)
    add_test(NAME live_reservation_tests COMMAND live_reservation_tests)
    add_executable(map_cache_tests test/map_cache_tests.cpp)
    target_link_libraries(map_cache_tests PRIVATE capability_mission_planner_map_adapter)
    add_test(NAME map_cache_tests COMMAND map_cache_tests)
  endif()
  if(TARGET offline_map_demo)
    if(BUILD_SCENARIO_TESTS)
      if(NOT IS_DIRECTORY "${SCENARIO_ROOT}")
        message(FATAL_ERROR
          "BUILD_SCENARIO_TESTS requires SCENARIO_ROOT pointing to capability_mission_scenarios")
      endif()
      add_executable(offline_adapter_tests test/offline_adapter_tests.cpp)
      target_link_libraries(offline_adapter_tests PRIVATE
        capability_mission_planner_map_adapter)
      add_test(NAME offline_adapter_tests COMMAND offline_adapter_tests
        ${SCENARIO_ROOT}/maps/zju2
        ${SCENARIO_ROOT}/maps/myj1)
      set(MYJ1_CONFIG ${SCENARIO_ROOT}/configs/myj1.yaml)
      if(NOT EXISTS "${MYJ1_CONFIG}")
        set(MYJ1_CONFIG ${SCENARIO_ROOT}/configs/myj1/myj1.yaml)
      endif()
      if(EXISTS "${MYJ1_CONFIG}")
        add_test(NAME offline_config_validation COMMAND capability_mission_planner_cli
          ${MYJ1_CONFIG}
          ${CMAKE_CURRENT_BINARY_DIR}/offline_config_output)
      endif()
      set(SINGLE_MAP_SCENARIOS
        01_single_generalist
        02_four_specialists
        03_overlapping_capabilities
        04_more_robots_than_tasks
        05_many_tasks_few_robots
        06_shared_sites)
      add_executable(offline_scenario_test test/offline_scenario_test.cpp)
      target_link_libraries(offline_scenario_test PRIVATE
        capability_mission_planner_map_adapter yaml-cpp)
      foreach(SCENARIO IN LISTS SINGLE_MAP_SCENARIOS)
        add_test(NAME single_map_${SCENARIO} COMMAND offline_scenario_test
          ${SCENARIO_ROOT}/configs/zju2/${SCENARIO}.yaml)
        set_tests_properties(single_map_${SCENARIO} PROPERTIES
          LABELS "single_map_scenarios")
      endforeach()
      add_test(NAME offline_single_map_validation COMMAND offline_map_demo
        ${SCENARIO_ROOT}/maps/zju2
        ${CMAKE_CURRENT_BINARY_DIR}/offline_single_map_output)
      add_test(NAME offline_multi_map_validation COMMAND offline_map_demo
        ${SCENARIO_ROOT}/maps/myj1
        ${CMAKE_CURRENT_BINARY_DIR}/offline_multi_map_output)
      if(EXISTS "${SCENARIO_ROOT}/configs/bdz1/01_three_corners_middle_corridors.yaml")
        add_test(NAME single_map_bdz1_three_corners COMMAND capability_mission_planner_cli
          ${SCENARIO_ROOT}/configs/bdz1/01_three_corners_middle_corridors.yaml
          ${CMAKE_CURRENT_BINARY_DIR}/offline_bdz1_output)
      endif()
      if(EXISTS "${SCENARIO_ROOT}/configs/bdz1/02_homogeneous_shortest_path_benchmark.yaml")
        add_test(NAME single_map_bdz1_homogeneous_benchmark COMMAND capability_mission_planner_cli
          ${SCENARIO_ROOT}/configs/bdz1/02_homogeneous_shortest_path_benchmark.yaml
          ${CMAKE_CURRENT_BINARY_DIR}/offline_bdz1_homogeneous_output)
      endif()
      if(EXISTS "${SCENARIO_ROOT}/configs/bdz1/03_homogeneous_shared_charger_benchmark.yaml")
        add_test(NAME single_map_bdz1_shared_charger_benchmark COMMAND capability_mission_planner_cli
          ${SCENARIO_ROOT}/configs/bdz1/03_homogeneous_shared_charger_benchmark.yaml
          ${CMAKE_CURRENT_BINARY_DIR}/offline_bdz1_shared_charger_output)
      endif()
    endif()
  endif()
endif()
