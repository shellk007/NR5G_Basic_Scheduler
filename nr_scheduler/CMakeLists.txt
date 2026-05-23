cmake_minimum_required(VERSION 3.16)
project(NexGenNRScheduler VERSION 1.0.0 LANGUAGES CXX)

# C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_BENCHMARKS "Build performance benchmarks" OFF)
option(ENABLE_SANITIZERS "Enable address/undefined sanitizers" OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wno-unused-parameter
        -Werror=return-type
    )
    
    if(ENABLE_SANITIZERS)
        add_compile_options(-fsanitize=address,undefined)
        add_link_options(-fsanitize=address,undefined)
    endif()
elseif(MSVC)
    add_compile_options(/W4 /permissive-)
endif()

# Release optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-O3 -march=native -flto)
        add_link_options(-flto)
    elseif(MSVC)
        add_compile_options(/O2 /GL)
        add_link_options(/LTCG)
    endif()
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Source files
set(SCHEDULER_SOURCES
    src/scheduler_engine.cpp
    src/qos_engine.cpp
    src/slice_orchestrator.cpp
)

# Header files (for IDE navigation)
set(SCHEDULER_HEADERS
    include/core/nr_types.hpp
    include/core/ue_context.hpp
    include/core/cell_context.hpp
    include/interfaces/phy_interface.hpp
    include/interfaces/mac_interface.hpp
    include/engine/scheduler_engine.hpp
    include/services/qos_engine.hpp
    include/services/slice_orchestrator.hpp
    include/channels/pdcch_manager.hpp
    include/channels/pdsch_manager.hpp
    include/channels/pusch_manager.hpp
    include/channels/pucch_manager.hpp
)

# Main library
add_library(nexgen_scheduler STATIC ${SCHEDULER_SOURCES} ${SCHEDULER_HEADERS})
target_include_directories(nexgen_scheduler PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Thread support
find_package(Threads REQUIRED)
target_link_libraries(nexgen_scheduler PUBLIC Threads::Threads)

# Installation
install(TARGETS nexgen_scheduler
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include/nexgen_scheduler
    FILES_MATCHING PATTERN "*.hpp"
)

# Tests
if(BUILD_TESTS)
    enable_testing()
    
    # Google Test
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    FetchContent_MakeAvailable(googletest)
    
    # Test executable
    add_executable(scheduler_tests
        tests/test_scheduler_engine.cpp
        tests/test_qos_engine.cpp
        tests/test_slice_orchestrator.cpp
        tests/test_resource_grid.cpp
    )
    
    target_link_libraries(scheduler_tests
        nexgen_scheduler
        GTest::gtest_main
    )
    
    include(GoogleTest)
    gtest_discover_tests(scheduler_tests)
endif()

# Benchmarks
if(BUILD_BENCHMARKS)
    find_package(benchmark REQUIRED)
    
    add_executable(scheduler_benchmarks
        benchmarks/benchmark_scheduling.cpp
        benchmarks/benchmark_resource_allocation.cpp
    )
    
    target_link_libraries(scheduler_benchmarks
        nexgen_scheduler
        benchmark::benchmark
    )
endif()

# Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_MAN NO)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_EXTRACT_PRIVATE YES)
    
    doxygen_add_docs(docs
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        COMMENT "Generating API documentation"
    )
endif()

# Print configuration
message(STATUS "")
message(STATUS "NexGen NR Scheduler Configuration:")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  CXX Compiler:   ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  Build tests:    ${BUILD_TESTS}")
message(STATUS "  Build benches:  ${BUILD_BENCHMARKS}")
message(STATUS "  Sanitizers:     ${ENABLE_SANITIZERS}")
message(STATUS "")
