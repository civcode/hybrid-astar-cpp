cmake_minimum_required(VERSION 3.20)
project(hybrid_astar_cpp VERSION 0.1.0 LANGUAGES CXX)

option(HYBRID_ASTAR_ENABLE_OMPL "Build OMPL-backed steering adapter" OFF)
option(HYBRID_ASTAR_BUILD_TESTS "Build standalone smoke tests" ON)
option(HYBRID_ASTAR_BUILD_BENCHMARKS "Build optional manually run planner benchmark" OFF)

add_library(hybrid_astar_core STATIC
  src/ackermann/search_config.cpp
  src/ackermann/planner.cpp
  src/steering/analytic_expansion.cpp)
add_library(hybrid_astar::core ALIAS hybrid_astar_core)
target_compile_features(hybrid_astar_core PUBLIC cxx_std_20)
target_include_directories(hybrid_astar_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(HYBRID_ASTAR_ENABLE_OMPL)
  find_package(ompl CONFIG REQUIRED)
  add_library(hybrid_astar_ompl STATIC src/steering/ompl_reeds_shepp.cpp)
  add_library(hybrid_astar::ompl ALIAS hybrid_astar_ompl)
  target_compile_features(hybrid_astar_ompl PUBLIC cxx_std_20)
  target_link_libraries(hybrid_astar_ompl PUBLIC hybrid_astar::core)
  if(TARGET ompl::ompl)
    target_link_libraries(hybrid_astar_ompl PRIVATE ompl::ompl)
  elseif(TARGET ompl)
    target_link_libraries(hybrid_astar_ompl PRIVATE ompl)
  elseif(OMPL_LIBRARIES)
    target_link_libraries(hybrid_astar_ompl PRIVATE ${OMPL_LIBRARIES})
    target_include_directories(hybrid_astar_ompl PRIVATE ${OMPL_INCLUDE_DIRS})
  else()
    message(FATAL_ERROR "OMPL was found without a known library target")
  endif()
endif()

if(HYBRID_ASTAR_BUILD_TESTS)
  enable_testing()
  add_executable(hybrid_astar_core_smoke tests/core_smoke.cpp)
  target_link_libraries(hybrid_astar_core_smoke PRIVATE hybrid_astar::core)
  add_test(NAME core_smoke COMMAND hybrid_astar_core_smoke)
  add_executable(hybrid_astar_planner_test tests/planner_test.cpp)
  target_link_libraries(hybrid_astar_planner_test PRIVATE hybrid_astar::core)
  add_test(NAME planner_test COMMAND hybrid_astar_planner_test)
  if(HYBRID_ASTAR_ENABLE_OMPL)
    add_executable(hybrid_astar_ompl_smoke tests/ompl_smoke.cpp)
    target_link_libraries(hybrid_astar_ompl_smoke PRIVATE hybrid_astar::ompl)
    add_test(NAME ompl_smoke COMMAND hybrid_astar_ompl_smoke)
  endif()
endif()

if(HYBRID_ASTAR_BUILD_BENCHMARKS)
  add_executable(hybrid_astar_planner_bench bench/planner_bench.cpp)
  target_link_libraries(hybrid_astar_planner_bench PRIVATE hybrid_astar::core)
endif()
