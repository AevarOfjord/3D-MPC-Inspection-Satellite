
cmake_minimum_required(VERSION 3.20)
project(satellite_control_cpp LANGUAGES CXX)


include(FetchContent)
FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(Eigen3)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(_cpp_mpc MODULE 
    src/satellite_control/cpp/bindings.cpp
    src/satellite_control/cpp/linearizer.cpp
)
target_link_libraries(_cpp_mpc PRIVATE pybind11::headers Python::Module Eigen3::Eigen)
target_compile_features(_cpp_mpc PRIVATE cxx_std_17)

# Avoid 'lib' prefix on Unix-like systems for Python modules
set_target_properties(_cpp_mpc PROPERTIES PREFIX "")

# Install into the package directory (relative to site-packages root)
install(TARGETS _cpp_mpc DESTINATION satellite_control/cpp)
