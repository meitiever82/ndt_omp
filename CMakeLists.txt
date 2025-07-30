cmake_minimum_required(VERSION 3.8)
project(ndt_omp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -mavx causes a lot of errors!!
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########
include_directories(include)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

ament_target_dependencies(ndt_omp
  rclcpp
  pcl_ros
  pcl_conversions
)

target_link_libraries(ndt_omp
  ${PCL_LIBRARIES}
)

add_executable(align
  apps/align.cpp
)

ament_target_dependencies(align
  rclcpp
  pcl_ros
  pcl_conversions
)

target_link_libraries(align
  ${PCL_LIBRARIES}
  ndt_omp
)

# Interactive viewer based on official PCL viewer
add_executable(interactive_viewer
  apps/interactive_viewer.cpp
)

ament_target_dependencies(interactive_viewer
  rclcpp
  pcl_ros
  pcl_conversions
)

target_link_libraries(interactive_viewer
  ${PCL_LIBRARIES}
)

############
## Install ##
############

install(TARGETS
  ndt_omp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  align
  interactive_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/
        DESTINATION include/
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(ndt_omp)
ament_export_dependencies(rclcpp pcl_ros pcl_conversions)

ament_package()
