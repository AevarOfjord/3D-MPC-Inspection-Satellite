cmake_minimum_required(VERSION 3.20)
project(SatelliteControl VERSION 0.1.0 LANGUAGES C CXX)

# --- Settings ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Suppress dev warnings for cleaner output
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL "Suppress dev" FORCE)

# --- Dependencies ---
include(FetchContent)

# Helper macro to fetch and patch legacy CMakeLists.txt (Version < 3.5)
macro(fetch_and_patch_legacy NAME REPO TAG)
    FetchContent_Declare(
        ${NAME}
        GIT_REPOSITORY ${REPO}
        GIT_TAG ${TAG}
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(${NAME})
    if(NOT ${NAME}_POPULATED)
        FetchContent_Populate(${NAME})
        message(STATUS "Patching ${NAME} CMakeLists.txt for CMake 4 compatibility...")
        set(TARGET_FILE "${${NAME}_SOURCE_DIR}/CMakeLists.txt")
        if(EXISTS "${TARGET_FILE}")
            file(READ ${TARGET_FILE} FILE_CONTENT)
            string(REGEX REPLACE "cmake_minimum_required\\(VERSION [0-9.]+\\)" "cmake_minimum_required(VERSION 3.10)" FILE_CONTENT "${FILE_CONTENT}")
            file(WRITE ${TARGET_FILE} "${FILE_CONTENT}")
        endif()
    endif()
    add_subdirectory(${${NAME}_SOURCE_DIR} ${${NAME}_BINARY_DIR} EXCLUDE_FROM_ALL)
endmacro()

message(STATUS "Fetching Dependencies...")

# 1. Eigen (Math) - Header only, modern enough usually
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Eigen)

# 2. yaml-cpp (Config Parsing)
set(YAML_CPP_BUILD_TESTS OFF CACHE INTERNAL "")
set(YAML_CPP_BUILD_TOOLS OFF CACHE INTERNAL "")
fetch_and_patch_legacy(yaml-cpp https://github.com/jbeder/yaml-cpp.git 0.8.0)

# 3. spdlog (Logging)
# Note: MuJoCo (via sdflib) fetches spdlog internally (v1.9.2).
# We omit our own fetch to avoid target name collision.
# FetchContent_Declare(
#   spdlog
#   GIT_REPOSITORY https://github.com/gabime/spdlog.git
#   GIT_TAG v1.12.0
#   GIT_SHALLOW TRUE
# )
# FetchContent_MakeAvailable(spdlog)

# 4. OSQP (Solver)
set(UNITTESTS OFF CACHE BOOL "" FORCE)
set(PRINTING ON CACHE BOOL "" FORCE)
set(PROFILING ON CACHE BOOL "" FORCE)
set(DLONG OFF CACHE BOOL "" FORCE)
# OSQP 0.6.3 needs patching for CMake < 3.5 issue
FetchContent_Declare(
    osqp
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG v0.6.3
    GIT_SHALLOW TRUE
)
FetchContent_GetProperties(osqp)
if(NOT osqp_POPULATED)
    FetchContent_Populate(osqp)
    # The fix: osqp has submodules (lin_sys/direct/qdldl...) that EACH have CMakeLists.txt < 3.5
    # We must patch them ALL recursively before adding subdirectory
    message(STATUS "Recursively patching OSQP CMakeLists.txt files with sed...")
    
    # We use 'find' and 'sed' because file(GLOB) is evaluated at configure time
    # and might miss files or fail to write before add_subdirectory is called.
    # This shell command is more robust for patching a downloaded tree.
    # Note: Regex allows for optional space before '(' and matches any version number
    execute_process(
        COMMAND find ${osqp_SOURCE_DIR} -name "CMakeLists.txt" -exec sed -i.bak "s/cmake_minimum_required.*/cmake_minimum_required(VERSION 3.10)/g" {} +
    )
endif()
add_subdirectory(${osqp_SOURCE_DIR} ${osqp_BINARY_DIR} EXCLUDE_FROM_ALL)

# 5. MuJoCo (Physics)
find_package(mujoco CONFIG)
if(NOT mujoco_FOUND)
  message(STATUS "MuJoCo not found installed, fetching from source...")
  # MuJoCo 3.1.1 is modern, should be fine
  FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG 3.1.1
    GIT_SHALLOW TRUE
  )
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(mujoco)
endif()

# --- Project Configuration ---
include_directories(include)

# List sources
set(SOURCES
    src/main.cpp
    src/io/ConfigLoader.cpp
    src/io/DataLogger.cpp
    src/physics/PhysicsEngine.cpp
    src/control/ControlAllocator.cpp
    src/control/MPCController.cpp
    src/logic/MissionManager.cpp
    src/fdir/FaultManager.cpp
    src/planner/MeshAnalyzer.cpp
    src/planner/InspectionPlanner.cpp
    src/export/FlightCodeGenerator.cpp
    src/hil/HILBridge.cpp
    src/telemetry/TelemetryServer.cpp
)

# --- Targets ---
add_executable(sat_control ${SOURCES})

# Handle OSQP library naming weirdness
if(TARGET osqp)
    set(OSQP_LIB osqp)
elseif(TARGET osqp::osqpstatic)
    set(OSQP_LIB osqp::osqpstatic)
elseif(TARGET osqpstatic)
    set(OSQP_LIB osqpstatic)
else()
    set(OSQP_LIB osqp)
endif()

target_link_libraries(sat_control PRIVATE
    Eigen3::Eigen
    yaml-cpp
    spdlog::spdlog
    ${OSQP_LIB}
    mujoco::mujoco
)

# Compiler Warnings
if(MSVC)
    target_compile_options(sat_control PRIVATE /W4)
else()
    target_compile_options(sat_control PRIVATE -Wall -Wextra -Wpedantic)
endif()
