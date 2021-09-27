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
find_package(SQLite3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/phmap)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EigenRand)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/morton/include)

# add optimization flags
option(WITH_VECTORIZATION "Build with vectorization" OFF) # currently disabled due to lack of optimization
include(CheckCXXCompilerFlag)
if (WITH_VECTORIZATION) 
    message(STATUS "Build with vectorization")
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
else(WITH_VECTORIZATION)
    # check BMI2 or AVX2 instructions for Morton code
    CHECK_CXX_COMPILER_FLAG("-mavx2" COMPILER_SUPPORTS_MAVX2)
    CHECK_CXX_COMPILER_FLAG("-mbmi2" COMPILER_SUPPORTS_MBMI2)
    if(COMPILER_SUPPORTS_MBMI2)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mbmi2")
        add_definitions(-D__BMI2__)
    elseif(COMPILER_SUPPORTS_MAVX2)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
        add_definitions(-D__AVX2__)
    else()
        message(FATAL_ERROR "AVX2 or BMI2 instruction support are mandatory!")
    endif()
endif(WITH_VECTORIZATION)

option(WITH_FASTMATH "Build with fast math optimization" ON)
if(WITH_FASTMATH)
    CHECK_CXX_COMPILER_FLAG("-ffast-math" COMPILER_SUPPORTS_FASTMATH)
    if(COMPILER_SUPPORTS_FASTMATH)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
    endif()
    add_definitions(-DEIGEN_FAST_MATH=1)
else()
    add_definitions(-DEIGEN_FAST_MATH=0)
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
            link_libraries(easy_profiler)
        else()
            add_definitions(-DDISABLE_EASY_PROFILER)
            message(STATUS "Profiler found but not enabled")
        endif()
    else ()
        message(STATUS "Profiler not found")
    endif ()
    add_definitions(-DPROFILING)
endif ()

option(EXPAND_PREDICTION "Use 27 neighbor blocks instead of 7 during prediction" OFF)
if (EXPAND_PREDICTION)
    add_definitions(-DEXPAND_PREDICTION)
endif ()

option(WITH_STACKTRACE "Build with stacktrace support from backward-cpp" ON)
if (WITH_STACKTRACE)
    add_subdirectory(thirdparty/backward-cpp)
    set(BACKWARD_SRC ${BACKWARD_ENABLE})
    if (STACK_DETAILS_DW)
        message(STATUS "Stacktrace using backward-cpp + libdw")
    elseif (STACK_DETAILS_DWARF)
        message(STATUS "Stacktrace using backward-cpp + libdwarf")
    elseif(STACK_DETAILS_BACKTRACE_SYMBOL)
        message(FATAL_ERROR "Stacktrace using backward-cpp")
    endif()
else()
    set(BACKWARD_SRC)
endif()

####################
## Catkin related ##
####################

# setup catkin packages
find_package(catkin_simple REQUIRED)
catkin_simple()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include/seddom
    include/dataset_devkit
    # ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${SQLite3_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bgkoctomap
#   src/${PROJECT_NAME}/bgkoctomap.cpp
# )


# add_executable(toy_example_node
#     src/toy_example_node.cpp
# )

# add_executable(kitti_node
#   src/kitti_node.cpp
# )

# add_executable(semantickitti_node
#     src/semantickitti_node.cpp
# )

cs_add_executable(octomap_server src/octomap_server.cpp ${BACKWARD_SRC})

# target_link_libraries(kitti_node
#   ${OpenCV_LIBRARIES}
# )

# target_link_libraries(semantickitti_node
#     ${OpenCV_LIBRARIES}
#     ${SQLite3_LIBRARIES}
# )

target_link_libraries(octomap_server
    ${SQLite3_LIBRARIES}
)

if (WITH_STACKTRACE)
    add_backward(octomap_server)
endif ()
