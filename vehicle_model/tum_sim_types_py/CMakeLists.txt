cmake_minimum_required(VERSION 3.5)
project(tum_sim_types_py)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Force FindPython3 to use the debug interpreter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

find_package(tum_types_cpp REQUIRED)
find_package(tum_sim_types_cpp REQUIRED)

# region target cpp bindings
ament_python_install_package(${PROJECT_NAME})

# region create target for cpp binding 
pybind11_add_module(_cpp_binding SHARED
  src/${PROJECT_NAME}/_cpp_binding.cpp
)
# ament_target_dependencies(_reindexer PUBLIC
#   "rosbag2_cpp"
#   "rosbag2_storage"
# )

# target_include_directories(_cpp_binding
#     PUBLIC 
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(_cpp_binding PUBLIC
  tum_types_cpp
  tum_sim_types_cpp
)
ament_export_dependencies(tum_types_cpp tum_sim_types_cpp)

 # Install cython modules as sub-modules of the project
install(
  TARGETS
    _cpp_binding
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
#endregion

ament_package()
