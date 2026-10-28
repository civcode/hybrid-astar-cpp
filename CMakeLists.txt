cmake_minimum_required(VERSION 3.20)
project(hybrid_astar_cpp VERSION 0.1.0 LANGUAGES CXX)

option(HYBRID_ASTAR_ENABLE_OMPL "Build OMPL-backed steering adapter" OFF)
option(HYBRID_ASTAR_BUILD_TESTS "Build Catch2 v3 unit and integration tests" ON)
option(HYBRID_ASTAR_USE_SYSTEM_CATCH2
  "Use installed Catch2 v3 instead of the default CMake FetchContent dependency" OFF)

if(HYBRID_ASTAR_BUILD_TESTS)
  if(HYBRID_ASTAR_USE_SYSTEM_CATCH2)
    find_package(Catch2 3 CONFIG REQUIRED)
  else()
    # Pinned upstream version. CMake downloads and builds this automatically
    # on first configure; existing FetchContent caches are reused.
    # Offline users may set FETCHCONTENT_SOURCE_DIR_CATCH2 to a local source tree.
    include(FetchContent)
    FetchContent_Declare(Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG        v3.16.1
      GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(Catch2)
  endif()
endif()
option(HYBRID_ASTAR_BUILD_BENCHMARKS "Build optional manually run planner benchmark" OFF)
option(HYBRID_ASTAR_REGISTER_REFINEMENT_BENCHMARK_TESTS
  "Opt in long-running Ackermann A/B benchmarks to CTest (requires benchmarks and tests)" OFF)
option(HYBRID_ASTAR_PROFILE_HOTPATHS "Collect opt-in timing of planner hot paths (adds clock overhead)" OFF)
option(HYBRID_ASTAR_BUILD_APP "Build the optional localhost browser planning demo" OFF)

add_library(hybrid_astar_core STATIC
  src/ackermann/search_config.cpp
  src/ackermann/planner.cpp
  src/ackermann/primitives.cpp
  src/ackermann/cover_prefilter.cpp
  src/ackermann/clearance.cpp
  src/ackermann/refinement.cpp
  src/ackermann/curved_shortcuts.cpp
  src/steering/analytic_expansion.cpp
  src/steering/signed_sections.cpp)
add_library(hybrid_astar::core ALIAS hybrid_astar_core)
add_library(hybrid_astar_differential_drive STATIC
  src/differential_drive/model.cpp src/differential_drive/primitives.cpp)
target_compile_features(hybrid_astar_differential_drive PUBLIC cxx_std_20)
target_include_directories(hybrid_astar_differential_drive PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
add_library(hybrid_astar::differential_drive ALIAS hybrid_astar_differential_drive)
add_library(hybrid_astar_mecanum STATIC
  src/mecanum/model.cpp src/mecanum/primitives.cpp)
target_compile_features(hybrid_astar_mecanum PUBLIC cxx_std_20)
target_include_directories(hybrid_astar_mecanum PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
add_library(hybrid_astar::mecanum ALIAS hybrid_astar_mecanum)
# Core search invokes differential primitive generation when that model is selected.
target_link_libraries(hybrid_astar_core PUBLIC hybrid_astar_differential_drive hybrid_astar_mecanum)
target_compile_features(hybrid_astar_core PUBLIC cxx_std_20)
if(HYBRID_ASTAR_PROFILE_HOTPATHS)
  target_compile_definitions(hybrid_astar_core PRIVATE HYBRID_ASTAR_PROFILE_HOTPATHS=1)
endif()
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
  add_executable(hybrid_astar_w12_mecanum_test tests/w12_mecanum_test.cpp)
  target_include_directories(hybrid_astar_w12_mecanum_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_w12_mecanum_test PRIVATE
    hybrid_astar::core hybrid_astar::mecanum Catch2::Catch2WithMain)
  add_test(NAME w12_mecanum_test COMMAND hybrid_astar_w12_mecanum_test)
  add_executable(hybrid_astar_w11_model_policy_test tests/w11_model_policy_test.cpp)
  target_link_libraries(hybrid_astar_w11_model_policy_test PRIVATE
    hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME w11_model_policy_test COMMAND hybrid_astar_w11_model_policy_test)
  add_executable(hybrid_astar_w10_differential_search_test tests/w10_differential_search_test.cpp)
  target_link_libraries(hybrid_astar_w10_differential_search_test PRIVATE
    hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME w10_differential_search_test COMMAND hybrid_astar_w10_differential_search_test)
  add_executable(hybrid_astar_differential_refinement_test tests/differential_refinement_test.cpp)
  target_link_libraries(hybrid_astar_differential_refinement_test PRIVATE
    hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME differential_refinement_test COMMAND hybrid_astar_differential_refinement_test)
  add_executable(hybrid_astar_w10_motion_primitives_test tests/w10_motion_primitives_test.cpp)
  target_link_libraries(hybrid_astar_w10_motion_primitives_test PRIVATE
    hybrid_astar::core hybrid_astar::differential_drive Catch2::Catch2WithMain)
  add_test(NAME w10_motion_primitives_test COMMAND hybrid_astar_w10_motion_primitives_test)
  add_executable(hybrid_astar_footprint_inflation_test tests/footprint_inflation_test.cpp)
  target_link_libraries(hybrid_astar_footprint_inflation_test PRIVATE
    hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME footprint_inflation_test COMMAND hybrid_astar_footprint_inflation_test)
  add_executable(hybrid_astar_hotpath_test tests/hotpath_test.cpp)
  target_include_directories(hybrid_astar_hotpath_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_hotpath_test PRIVATE hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME hotpath_test COMMAND hybrid_astar_hotpath_test)
  add_executable(hybrid_astar_priority_queue_test tests/priority_queue_test.cpp)
  target_include_directories(hybrid_astar_priority_queue_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_priority_queue_test PRIVATE hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME priority_queue_test COMMAND hybrid_astar_priority_queue_test)
  add_executable(hybrid_astar_hard_scenario_test tests/hard_scenario_test.cpp)
  target_include_directories(hybrid_astar_hard_scenario_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_hard_scenario_test PRIVATE hybrid_astar::core Catch2::Catch2WithMain)
  if(HYBRID_ASTAR_ENABLE_OMPL)
    target_link_libraries(hybrid_astar_hard_scenario_test PRIVATE hybrid_astar::ompl)
    target_compile_definitions(hybrid_astar_hard_scenario_test PRIVATE HYBRID_ASTAR_HARD_CASES_WITH_OMPL=1)
  endif()
  add_test(NAME hard_scenario_test COMMAND hybrid_astar_hard_scenario_test)
  add_executable(hybrid_astar_catch2_contract_test tests/catch2_contract_test.cpp)
  target_link_libraries(hybrid_astar_catch2_contract_test PRIVATE hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME catch2_contract_test COMMAND hybrid_astar_catch2_contract_test)
  add_executable(hybrid_astar_core_smoke tests/core_smoke.cpp)
  target_link_libraries(hybrid_astar_core_smoke PRIVATE hybrid_astar::core)
  add_test(NAME core_smoke COMMAND hybrid_astar_core_smoke)
  add_executable(hybrid_astar_planner_test tests/planner_test.cpp)
  target_link_libraries(hybrid_astar_planner_test PRIVATE hybrid_astar::core)
  add_test(NAME planner_test COMMAND hybrid_astar_planner_test)
  add_executable(hybrid_astar_signed_sections_test tests/signed_sections_test.cpp)
  target_link_libraries(hybrid_astar_signed_sections_test PRIVATE hybrid_astar::core)
  add_test(NAME signed_sections_test COMMAND hybrid_astar_signed_sections_test)
  add_executable(hybrid_astar_scenarios_test tests/scenarios_test.cpp)
  target_link_libraries(hybrid_astar_scenarios_test PRIVATE hybrid_astar::core)
  add_test(NAME scenarios_test COMMAND hybrid_astar_scenarios_test)
  add_executable(hybrid_astar_refinement_test tests/refinement_test.cpp)
  target_link_libraries(hybrid_astar_refinement_test PRIVATE hybrid_astar::core)
  add_test(NAME refinement_test COMMAND hybrid_astar_refinement_test)
  add_executable(hybrid_astar_steering_objective_test tests/steering_objective_test.cpp)
  target_link_libraries(hybrid_astar_steering_objective_test PRIVATE
    hybrid_astar::core Catch2::Catch2WithMain)
  add_test(NAME steering_objective_test COMMAND hybrid_astar_steering_objective_test)
  add_executable(hybrid_astar_curved_shortcuts_test tests/curved_shortcuts_test.cpp)
  target_link_libraries(hybrid_astar_curved_shortcuts_test PRIVATE hybrid_astar::core)
  add_test(NAME curved_shortcuts_test COMMAND hybrid_astar_curved_shortcuts_test)
  add_executable(hybrid_astar_quality_test tests/quality_test.cpp)
  target_link_libraries(hybrid_astar_quality_test PRIVATE hybrid_astar::core)
  add_test(NAME quality_test COMMAND hybrid_astar_quality_test)
  add_executable(hybrid_astar_prefilter_test tests/prefilter_test.cpp)
  target_link_libraries(hybrid_astar_prefilter_test PRIVATE hybrid_astar::core)
  add_test(NAME prefilter_test COMMAND hybrid_astar_prefilter_test)
  add_executable(hybrid_astar_clearance_test tests/clearance_test.cpp)
  target_link_libraries(hybrid_astar_clearance_test PRIVATE hybrid_astar::core)
  add_test(NAME clearance_test COMMAND hybrid_astar_clearance_test)
  if(HYBRID_ASTAR_ENABLE_OMPL)
    add_executable(hybrid_astar_ompl_smoke tests/ompl_smoke.cpp)
    target_link_libraries(hybrid_astar_ompl_smoke PRIVATE hybrid_astar::ompl)
    add_test(NAME ompl_smoke COMMAND hybrid_astar_ompl_smoke)
    add_executable(hybrid_astar_ompl_integration_test tests/ompl_integration_test.cpp)
    target_link_libraries(hybrid_astar_ompl_integration_test PRIVATE hybrid_astar::ompl)
    add_test(NAME ompl_integration_test COMMAND hybrid_astar_ompl_integration_test)
  endif()
endif()

if(HYBRID_ASTAR_BUILD_BENCHMARKS)
  add_executable(hybrid_astar_hotpath_bench bench/hotpath_bench.cpp)
  target_include_directories(hybrid_astar_hotpath_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_hotpath_bench PRIVATE hybrid_astar::core)
  add_executable(hybrid_astar_priority_queue_bench bench/priority_queue_bench.cpp)
  target_include_directories(hybrid_astar_priority_queue_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_priority_queue_bench PRIVATE hybrid_astar::core)
  add_executable(hybrid_astar_hard_scenario_bench bench/hard_scenario_bench.cpp)
  target_include_directories(hybrid_astar_hard_scenario_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(hybrid_astar_hard_scenario_bench PRIVATE hybrid_astar::core)
  if(HYBRID_ASTAR_ENABLE_OMPL)
    target_link_libraries(hybrid_astar_hard_scenario_bench PRIVATE hybrid_astar::ompl)
    target_compile_definitions(hybrid_astar_hard_scenario_bench PRIVATE HYBRID_ASTAR_HARD_CASES_WITH_OMPL=1)
  endif()
  add_executable(hybrid_astar_planner_bench bench/planner_bench.cpp)
  target_link_libraries(hybrid_astar_planner_bench PRIVATE hybrid_astar::core)
  add_executable(hybrid_astar_refinement_bench bench/refinement_bench.cpp)
  target_link_libraries(hybrid_astar_refinement_bench PRIVATE hybrid_astar::core)

  foreach(_variant IN ITEMS budget smoothness)
    add_executable(hybrid_astar_ackermann_refinement_${_variant}_benchmark_test
      tests/ackermann_refinement_${_variant}_benchmark_test.cpp)
    target_include_directories(hybrid_astar_ackermann_refinement_${_variant}_benchmark_test
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(hybrid_astar_ackermann_refinement_${_variant}_benchmark_test
      PRIVATE hybrid_astar::core)
    if(HYBRID_ASTAR_ENABLE_OMPL)
      target_link_libraries(hybrid_astar_ackermann_refinement_${_variant}_benchmark_test
        PRIVATE hybrid_astar::ompl)
      target_compile_definitions(hybrid_astar_ackermann_refinement_${_variant}_benchmark_test
        PRIVATE HYBRID_ASTAR_REFINEMENT_BENCH_WITH_OMPL=1)
    endif()
    if(HYBRID_ASTAR_BUILD_TESTS AND HYBRID_ASTAR_REGISTER_REFINEMENT_BENCHMARK_TESTS)
      add_test(NAME ackermann_refinement_${_variant}_benchmark
        COMMAND hybrid_astar_ackermann_refinement_${_variant}_benchmark_test
          5 10000 100 250000 3000)
      set_tests_properties(ackermann_refinement_${_variant}_benchmark
        PROPERTIES LABELS "benchmark;ackermann")
    endif()
  endforeach()
endif()

# Developer demo only: standalone browser UI served exclusively on loopback.
# No ROS dependency, no externally hosted service, no GitHub Actions.
if(HYBRID_ASTAR_BUILD_APP)
  add_library(hybrid_astar_demo_logic STATIC apps/demo/demo.cpp apps/demo/map_io.cpp apps/demo/map_yaml.cpp apps/demo/map_edits.cpp)
  target_compile_features(hybrid_astar_demo_logic PUBLIC cxx_std_20)
  target_include_directories(hybrid_astar_demo_logic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(hybrid_astar_demo_logic PUBLIC hybrid_astar::core)
  if(HYBRID_ASTAR_ENABLE_OMPL)
    target_link_libraries(hybrid_astar_demo_logic PUBLIC hybrid_astar::ompl)
    target_compile_definitions(hybrid_astar_demo_logic PRIVATE HYBRID_ASTAR_DEMO_WITH_OMPL=1)
  endif()
  add_executable(hybrid_astar_demo apps/demo/main.cpp)
  target_link_libraries(hybrid_astar_demo PRIVATE hybrid_astar_demo_logic)
  target_compile_definitions(hybrid_astar_demo PRIVATE
    HYBRID_ASTAR_DEMO_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/web")
  if(HYBRID_ASTAR_BUILD_TESTS)
    add_executable(hybrid_astar_demo_test tests/demo_test.cpp)
    target_link_libraries(hybrid_astar_demo_test PRIVATE hybrid_astar_demo_logic)
    add_test(NAME demo_test COMMAND hybrid_astar_demo_test)
    add_executable(hybrid_astar_demo_map_io_test tests/demo_map_io_test.cpp)
    target_link_libraries(hybrid_astar_demo_map_io_test PRIVATE hybrid_astar_demo_logic Catch2::Catch2WithMain)
    add_test(NAME demo_map_io_test COMMAND hybrid_astar_demo_map_io_test)
    add_executable(hybrid_astar_demo_map_yaml_test tests/demo_map_yaml_test.cpp)
    target_link_libraries(hybrid_astar_demo_map_yaml_test PRIVATE hybrid_astar_demo_logic Catch2::Catch2WithMain)
    target_compile_definitions(hybrid_astar_demo_map_yaml_test PRIVATE
      HYBRID_ASTAR_SAMPLE_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/maps/sample.yaml")
    add_test(NAME demo_map_yaml_test COMMAND hybrid_astar_demo_map_yaml_test)
    add_executable(hybrid_astar_demo_map_edit_test tests/demo_map_edit_test.cpp)
    target_link_libraries(hybrid_astar_demo_map_edit_test PRIVATE hybrid_astar_demo_logic Catch2::Catch2WithMain)
    add_test(NAME demo_map_edit_test COMMAND hybrid_astar_demo_map_edit_test)
    add_test(NAME demo_cli_import_yaml COMMAND hybrid_astar_demo
      --once --map-yaml ${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/maps/sample.yaml)
    find_program(HYBRID_ASTAR_NODE_EXE node)
    if(HYBRID_ASTAR_NODE_EXE)
      add_test(NAME demo_map_editor_test COMMAND ${HYBRID_ASTAR_NODE_EXE}
        ${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/web/map_editor.test.js)
      add_test(NAME demo_map_geometry_test COMMAND ${HYBRID_ASTAR_NODE_EXE}
        ${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/web/map_geometry.test.js)
    endif()
    add_test(NAME demo_cli_open COMMAND hybrid_astar_demo --once --scenario open)
    add_test(NAME demo_cli_certified COMMAND hybrid_astar_demo
      --once --scenario open --certified)
    add_test(NAME demo_cli_budget COMMAND hybrid_astar_demo
      --once --scenario warehouse --budget 60000)
    add_test(NAME demo_cli_budget_rejected COMMAND hybrid_astar_demo
      --once --scenario warehouse --budget 7)
    set_tests_properties(demo_cli_budget_rejected PROPERTIES
      PASS_REGULAR_EXPRESSION "Budget must be an integer between 100 and 1000000")
    add_test(NAME demo_cli_differential_spin COMMAND hybrid_astar_demo
      --once --model differential --scenario open
      --start 10 15 0 --goal 10 15 90)
    add_test(NAME demo_cli_import_pgm COMMAND hybrid_astar_demo
      --once --map ${CMAKE_CURRENT_SOURCE_DIR}/apps/demo/maps/sample.pgm
      --resolution 0.5)
    add_test(NAME demo_cli_docker_bind COMMAND hybrid_astar_demo
      --once --bind 0.0.0.0 --scenario reverse)
    if(HYBRID_ASTAR_ENABLE_OMPL)
      add_test(NAME demo_cli_ompl COMMAND hybrid_astar_demo
        --once --scenario parking --ompl --refine)
    endif()
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND AND UNIX)
      add_test(NAME demo_http_test COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/demo_http_test.py $<TARGET_FILE:hybrid_astar_demo>)
    endif()
  endif()
endif()

# Catch2 v3 is the actual assertion/runner framework for every C++ test.
# Keep the existing named CTest suites for stable local invocation.
if(HYBRID_ASTAR_BUILD_TESTS)
  foreach(_case IN ITEMS
      hybrid_astar_core_smoke hybrid_astar_planner_test
      hybrid_astar_quality_test hybrid_astar_prefilter_test
      hybrid_astar_clearance_test hybrid_astar_signed_sections_test
      hybrid_astar_scenarios_test hybrid_astar_refinement_test
      hybrid_astar_curved_shortcuts_test hybrid_astar_demo_test
      hybrid_astar_ompl_smoke hybrid_astar_ompl_integration_test)
    if(TARGET ${_case})
      target_link_libraries(${_case} PRIVATE Catch2::Catch2WithMain)
    endif()
  endforeach()
endif()
