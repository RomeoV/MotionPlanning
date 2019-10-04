cmake_minimum_required(VERSION 3.10)
project(PathPlanner)

# set(CMAKE_CXX_FLAGS="-ggdb")

find_package(Python3 COMPONENTS Development NumPy)

add_library(cppitertools INTERFACE)
target_include_directories(cppitertools INTERFACE external)

add_library(matplotlib-cpp INTERFACE)
target_include_directories(matplotlib-cpp INTERFACE external)
target_link_libraries(matplotlib-cpp INTERFACE Python3::Python Python3::NumPy)

add_subdirectory(external/eigen)
add_subdirectory(external/range-v3)
add_subdirectory(external/fmt)
add_subdirectory(external/cxxopts)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")
find_package(TBB REQUIRED)

find_package(Boost REQUIRED COMPONENTS graph)

add_library(PathPlanner
  src/path_planner.cpp
)
target_link_libraries(PathPlanner
  TBB::tbb
  Boost::graph
  cppitertools
  fmt
  range-v3
  eigen
  matplotlib-cpp
  cxxopts
)
target_include_directories(PathPlanner
  PUBLIC
  src
)
set_target_properties(PathPlanner
  PROPERTIES
  CXX_STANDARD 17
)

add_executable(run
  src/main.cpp
)
target_link_libraries(run
  PathPlanner
)
set_target_properties(run
  PROPERTIES
  CXX_STANDARD 17
)

enable_testing()
# add_subdirectory(test)