cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# Add the ControlIt!-specific build options and macros
rosbuild_find_ros_package(controlit_cmake)
list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_PACKAGE_PATH}/cmake)
include(controlitbuild)
controlit_build_init()

# find Eigen/RBDL
controlit_build_depend_rbdl()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


file(GLOB SRCS src/*.cpp)
rosbuild_add_library (${PROJECT_NAME} SHARED ${SRCS})

rosbuild_add_openmp_flags(${PROJECT_NAME})
rosbuild_link_boost(${PROJECT_NAME} signals)
controlit_build_link_depends(${PROJECT_NAME})

# TESTS!
# add_subdirectory(tests)
