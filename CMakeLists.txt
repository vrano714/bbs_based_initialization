cmake_minimum_required(VERSION 3.5)
project(bbs_based_initializer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gpu_bbs3d REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

# CUDA
if(BUILD_CUDA)
  ament_auto_add_executable(bbs_based_initializer
    src/bbs_based_initializer/bbs_based_initializer.cpp
    src/bbs_based_initializer/bbs_based_initializer_node.cpp
  )

  target_include_directories(bbs_based_initializer
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    # ${Iridescence_INCLUDE_DIRS}
  )

  target_link_libraries(bbs_based_initializer
    ${PCL_LIBRARIES}
    yaml-cpp
    ${gpu_bbs3d_LIBRARY}
    # ${Iridescence_LIBRARIES}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)