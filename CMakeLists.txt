
cmake_minimum_required(VERSION 3.20)
project(satellite_control_cpp LANGUAGES CXX)


include(FetchContent)

# Eigen3 for linear algebra
FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
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
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# --- Module: _cpp_mpc ---
add_library(_cpp_mpc MODULE 
    src/satellite_control/cpp/bindings.cpp
    src/satellite_control/cpp/linearizer.cpp
    src/satellite_control/cpp/mpc_controller.cpp
)
target_link_libraries(_cpp_mpc PRIVATE pybind11::headers Python::Module Eigen3::Eigen osqpstatic)
target_include_directories(_cpp_mpc PRIVATE ${osqp_SOURCE_DIR}/include)
target_compile_features(_cpp_mpc PRIVATE cxx_std_17)
set_target_properties(_cpp_mpc PROPERTIES PREFIX "")
install(TARGETS _cpp_mpc DESTINATION satellite_control/cpp)

# Helper macro for future modules
macro(add_satellite_module name sources)
    add_library(${name} MODULE ${sources} ${ARGN})
    target_link_libraries(${name} PRIVATE pybind11::headers Python::Module Eigen3::Eigen)
    target_compile_features(${name} PRIVATE cxx_std_17)
    set_target_properties(${name} PROPERTIES PREFIX "")
    install(TARGETS ${name} DESTINATION satellite_control/cpp)
endmacro()

# Module: _cpp_sim
add_satellite_module(_cpp_sim
    src/satellite_control/cpp/bindings_sim.cpp
    src/satellite_control/cpp/simulation_engine.cpp
    src/satellite_control/cpp/orbital_dynamics.cpp
)

# Module: _cpp_physics
add_satellite_module(_cpp_physics
    src/satellite_control/cpp/bindings_physics.cpp
    src/satellite_control/cpp/orbital_dynamics.cpp
)
