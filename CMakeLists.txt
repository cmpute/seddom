cmake_minimum_required(VERSION 2.8.3)
project(seddom)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
            "Default build type: RelWithDebInfo" FORCE)
endif()
message(STATUS "Build config: ${CMAKE_BUILD_TYPE}")

find_package(OpenCV REQUIRED)
find_package(msgpack REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common filters)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/phmap)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EigenRand)

# add optimization flags
if (FALSE) # currently disabled due to lack of optimization
    message(STATUS "Build with vectorization")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
    if(COMPILER_SUPPORTS_MARCH_NATIVE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    endif()
    CHECK_CXX_COMPILER_FLAG("-mtune=native" COMPILER_SUPPORTS_MTUNE_NATIVE)
    if(COMPILER_SUPPORTS_MARCH_NATIVE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
    endif()
    CHECK_CXX_COMPILER_FLAG("-xHost" COMPILER_SUPPORTS_INTEL_XHOST)
    if(COMPILER_SUPPORTS_INTEL_XHOST)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xHost")
    endif()
    add_definitions(-DPCL_ENABLE_SSE)
    add_definitions(-DPCL_NO_PRECOMPILE)
endif()

# enable OpenMP
option(WITH_OPENMP "Build with openmp support!" ON)
if (WITH_OPENMP)
  find_package(OpenMP)
  if (OPENMP_FOUND)
      set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      add_definitions(-DOPENMP)
      message(STATUS "OpenMP enabled")
  else()
      message(STATUS "OpenMP not found")
  endif()
endif()

option(WITH_PROFILER "Build with profiler" ON)
if (WITH_PROFILER)
    find_package(easy_profiler)
    if (easy_profiler_FOUND)
        if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
            message(STATUS "Profiler enabled")
        else()
            add_definitions(-DDISABLE_EASY_PROFILER)
            message(STATUS "Profiler found but not enabled")
        endif()
    else ()
        message(STATUS "Profiler not found")
    endif ()
endif ()

option(EXPAND_PREDICTION "Use 27 neighbor blocks instead of 7 during prediction" OFF)
if (EXPAND_PREDICTION)
    add_definitions(-DEXPAND_PREDICTION)
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        visualization_msgs
        nav_msgs
        pcl_ros
        roscpp
        tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

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
#  CATKIN_DEPENDS geometry_msgs pcl_ros roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include/seddom
    include/dataset_devkit
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bgkoctomap
#   src/${PROJECT_NAME}/bgkoctomap.cpp
# )

# Files for SEMANTIC BGK source
set(MAPPING_SOURCE_FILES
    # src/mapping/bkiblock.cpp
    # src/mapping/bkioctomap.cpp
    # src/mapping/bkioctree_node.cpp
    # src/mapping/bkioctree.cpp
)


# add_executable(toy_example_node
#     ${MAPPING_SOURCE_FILES}
#     ${COMMON_SOURCE_FILES}
#     src/toy_example_node.cpp
# )

# add_executable(kitti_node
#   ${MAPPING_SOURCE_FILES}
#   ${COMMON_SOURCE_FILES}
#   src/kitti_node.cpp
# )

add_executable(semantickitti_node
    ${MAPPING_SOURCE_FILES}
    src/semantickitti_node.cpp
)

add_executable(octomap_server
    ${MAPPING_SOURCE_FILES}
    src/octomap_server.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(BGKOctoMap_node BGKOctoMap_generate_messages_cpp)

## Specify libraries to link a library or executable target against

# target_link_libraries(toy_example_node
#     ${catkin_LIBRARIES}
#     easy_profiler
# )

# target_link_libraries(kitti_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   easy_profiler
# )

target_link_libraries(semantickitti_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    easy_profiler
)

target_link_libraries(octomap_server
    ${catkin_LIBRARIES}
    easy_profiler
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bgkoctomap BGKOctoMap_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_BGKOctoMap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
