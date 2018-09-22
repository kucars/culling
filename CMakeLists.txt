cmake_minimum_required(VERSION 2.8.3)
project(culling)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
#add_definitions(-std=c++11)

#find_package(octomap REQUIRED)
include("/usr/local/share/octomap/octomap-config.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  rviz_visual_tools
  #octomap_world
)

FIND_PACKAGE(CUDA)
find_package(Boost REQUIRED)
# CGAL and its components
#find_package(CGAL QUIET COMPONENTS)

if (OPENMP_FOUND)
    message(STATUS "*************************************************************************************************")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES ${OCTOMAP_LIBRARIES} ${OCTOMAP_LIBRARIES_DIR}
  CATKIN_DEPENDS roscpp pcl_ros 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

if (CUDA_FOUND)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    #-gencode arch=compute_61,code=sm_61
    #-gencode arch=compute_62,code=sm_62
  )
  #add_definitions(-DGPU)
  cuda_add_library(frustum_culling_gpu src/frustum_culling_gpu.cu)
  cuda_add_library(voxel_grid_occlusion_estimation_gpu src/voxel_grid_occlusion_estimation_gpu.cu)
  add_library(occlusion_culling_gpu src/occlusion_culling_gpu.cpp)
  target_link_libraries(occlusion_culling_gpu frustum_culling_gpu voxel_grid_occlusion_estimation_gpu ${catkin_LIBRARIES} ${PCL_LIBRARIES})

endif()

add_executable(occlusion_culling_test 
              src/occlusion_culling_test.cpp 
              src/occlusion_culling.cpp
              src/voxel_grid_occlusion_estimation.cpp
              src/frustum_culling.cpp)
add_dependencies(occlusion_culling_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(occlusion_culling_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})

#add_executable (voxel_grid_occlusion_estimation_test src/voxel_grid_occlusion_estimation_test.cpp)
#target_link_libraries(voxel_grid_occlusion_estimation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})
