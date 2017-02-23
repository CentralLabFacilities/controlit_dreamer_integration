cmake_minimum_required(VERSION 2.8.3)
project(controlit_dreamer_integration)

################
## ControlIt! ##
################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__IN_RTAI__ -ffast-math")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__IN_RTAI__ -ffast-math")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    controlit_cmake
    controlit_core
    roscpp
)

#message("** controlit_cmake_DIR: " ${controlit_cmake_DIR})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../controlit/controlit_cmake/Modules ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)
#message("** CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

include(controlitbuild)
controlit_build_init()

find_package(Eigen REQUIRED)
find_package(OpenMP)
find_package(LibSerial REQUIRED)
find_package(RTAI REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# # Add the ControlIt!-specific build options and macros
# rosbuild_find_ros_package(controlit_cmake)
# list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_PACKAGE_PATH}/cmake)
# include(controlitbuild)
# controlit_build_init()

# # find Eigen/RBDL
# controlit_build_depend_rbdl()

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###########
## Build ##
###########

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "+++ ${_variableName}=${${_variableName}}")
# endforeach()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${LIBSERIAL_INCLUDE_DIR}
    ${RTAI_INCLUDE_DIR}
    ${controlit_core_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/OdometryStateReceiverDreamer.cpp
    src/PluginList.cpp
    src/RobotInterfaceDreamer.cpp
    src/ServoClockDreamer.cpp
    src/HandControllerDreamer.cpp
    src/HeadControllerDreamer.cpp
    src/TimerRTAI.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${LIBSERIAL_LIBRARY}
    ${RTAI_LIBRARY}
    ${controlit_core_LIBRARIES}
)

add_executable(ServoClockDreamerTester src/ServoClockDreamerTester.cpp)

target_link_libraries(ServoClockDreamerTester
    ${PROJECT_NAME}
    ${controlit_core_LIBRARIES}
)

add_executable(RobotInterfaceDreamerTester src/RobotInterfaceDreamerTester.cpp)

target_link_libraries(RobotInterfaceDreamerTester
    ${PROJECT_NAME}
    ${controlit_core_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(DIRECTORY launch parameters plugins/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# TESTS!
# add_subdirectory(tests)
