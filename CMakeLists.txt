
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

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(_cpp_mpc MODULE 
    src/satellite_control/cpp/bindings.cpp
    src/satellite_control/cpp/linearizer.cpp
    src/satellite_control/cpp/mpc_controller.cpp
)
target_link_libraries(_cpp_mpc PRIVATE pybind11::headers Python::Module Eigen3::Eigen osqpstatic)
target_include_directories(_cpp_mpc PRIVATE ${osqp_SOURCE_DIR}/include)
target_compile_features(_cpp_mpc PRIVATE cxx_std_17)

# Avoid 'lib' prefix on Unix-like systems for Python modules
set_target_properties(_cpp_mpc PROPERTIES PREFIX "")

# Install into the package directory (relative to site-packages root)
install(TARGETS _cpp_mpc DESTINATION satellite_control/cpp)
