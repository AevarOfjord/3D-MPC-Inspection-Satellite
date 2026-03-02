
cmake_minimum_required(VERSION 3.20)
project(satellite_control_cpp LANGUAGES CXX)


include(FetchContent)

# Eigen3 for linear algebra (disable tests/docs for faster builds)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  # Fix for hanging git ls-remote on local directory during configure
  PATCH_COMMAND sed -i.bak "s/execute_process(COMMAND git ls-remote/# execute_process(COMMAND git ls-remote/" CMakeLists.txt
)
FetchContent_MakeAvailable(Eigen3)

# OSQP solver
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v0.6.3
)
set(OSQP_BUILD_SHARED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(osqp)

# Common Dependencies
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Python3_EXECUTABLE: ${Python3_EXECUTABLE}")
message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
find_package(pybind11 CONFIG REQUIRED)

# --- MPC profile modules (RTI-SQP, CasADi + OSQP) ---
function(add_mpc_profile_module name)
    add_library(${name} MODULE ${ARGN})
    target_link_libraries(${name} PRIVATE pybind11::headers Python3::Module Eigen3::Eigen osqpstatic)
    target_include_directories(${name} PRIVATE
        ${osqp_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/controller/shared/cpp
    )
    target_compile_features(${name} PRIVATE cxx_std_17)
    set_target_properties(${name} PROPERTIES PREFIX "")
    install(TARGETS ${name} DESTINATION controller/shared/python/cpp)
endfunction()

add_mpc_profile_module(_cpp_mpc
    controller/hybrid/cpp/bindings.cpp
    controller/hybrid/cpp/sqp_controller.cpp
    controller/hybrid/cpp/sqp_types.cpp
)

add_mpc_profile_module(_cpp_mpc_nonlinear
    controller/nonlinear/cpp/bindings.cpp
    controller/nonlinear/cpp/sqp_controller.cpp
    controller/hybrid/cpp/sqp_controller.cpp
    controller/hybrid/cpp/sqp_types.cpp
)

add_mpc_profile_module(_cpp_mpc_linear
    controller/linear/cpp/bindings.cpp
    controller/linear/cpp/sqp_controller.cpp
    controller/hybrid/cpp/sqp_controller.cpp
    controller/hybrid/cpp/sqp_types.cpp
)

# Helper macro for future modules
macro(add_satellite_module name sources)
    add_library(${name} MODULE ${sources} ${ARGN})
    target_link_libraries(${name} PRIVATE pybind11::headers Python3::Module Eigen3::Eigen)
    target_compile_features(${name} PRIVATE cxx_std_17)
    set_target_properties(${name} PROPERTIES PREFIX "")
    install(TARGETS ${name} DESTINATION controller/shared/python/cpp)
endmacro()

# Module: _cpp_sim
add_satellite_module(_cpp_sim
    controller/shared/cpp/sim/bindings_sim.cpp
    controller/shared/cpp/sim/simulation_engine.cpp
    controller/shared/cpp/sim/orbital_dynamics.cpp
)

# Module: _cpp_physics
add_satellite_module(_cpp_physics
    controller/shared/cpp/sim/bindings_physics.cpp
    controller/shared/cpp/sim/orbital_dynamics.cpp
)
