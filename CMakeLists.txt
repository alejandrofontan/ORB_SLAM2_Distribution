cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_Distribution)

option(COMPILE_WITH_PANGOLIN "Compile using Pangolin for Visualization" ON)
option(COMPILE_SEQUENTIAL "Compile Sequential Version Of ORB_SLAM2" OFF)
option(COMPILE_DETERMINISTIC "Compile Deterministic Version Of ORB_SLAM2" OFF)
option(COMPILE_DEBUG "Compile DEBUG Version Of ORB_SLAM2" OFF)
option(COMPILE_ABLATION "Compile ABLATION Version Of ORB_SLAM2" OFF)

if(COMPILE_WITH_PANGOLIN)
    add_definitions(-DCOMPILED_WITH_PANGOLIN=1)
endif()

if(COMPILE_SEQUENTIAL)
    add_definitions(-DCOMPILED_SEQUENTIAL=1)
endif()

if(COMPILE_DETERMINISTIC)
    add_definitions(-DCOMPILED_DETERMINISTIC=1)
endif()

if(COMPILE_DEBUG)
    add_definitions(-DCOMPILED_DEBUG=1)
endif()

if(COMPILE_ABLATION)
    add_definitions(-DCOMPILED_ABLATION=1)
endif()

set(OPENCV_LOCAL_VERSION "4.7.0")
set(EIGEN_LOCAL_VERSION "3.4.0")
set(PANGOLIN_LOCAL_VERSION "0.8")

set(OPENCV_LOCAL_PATH "/home/alex/libraries/opencv-${OPENCV_LOCAL_VERSION}/install")
set(EIGEN_LOCAL_PATH "/home/alex/libraries/eigen-${EIGEN_LOCAL_VERSION}/install")
set(PANGOLIN_LOCAL_PATH "/home/alex/libraries/Pangolin-${PANGOLIN_LOCAL_VERSION}")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++17
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using flag -std=c++17.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV ${OPENCV_LOCAL_VERSION} REQUIRED HINTS ${OPENCV_LOCAL_PATH})
find_package(Eigen3 ${EIGEN_LOCAL_VERSION} REQUIRED HINTS ${EIGEN_LOCAL_PATH})
find_package(GSL REQUIRED)

if(COMPILE_WITH_PANGOLIN)
    find_package(Pangolin ${PANGOLIN_LOCAL_VERSION} REQUIRED HINTS ${PANGOLIN_LOCAL_PATH})
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${GSL_INCLUDE_DIRS}
)

if(COMPILE_WITH_PANGOLIN)
    include_directories(${Pangolin_INCLUDE_DIRS})
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/Utils.cpp
src/SLAMGraph.cpp
src/DistributionFitter.cpp)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${GSL_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
if(COMPILE_WITH_PANGOLIN)
    target_link_libraries(
            ${PROJECT_NAME}
            ${Pangolin_LIBRARIES}
    )
endif()

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/rgbd)

add_executable(rgbd_tum
Examples/rgbd/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/stereo)

add_executable(stereo_kitti
Examples/stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/mono)

add_executable(mono_rgbdtum
        Examples/mono/mono_rgbdtum.cc)
target_link_libraries(mono_rgbdtum ${PROJECT_NAME})

add_executable(mono_monotum
        Examples/mono/mono_monotum.cc)
target_link_libraries(mono_monotum ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/mono/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/mono/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_vector
        Examples/mono/mono_vector.cc)
target_link_libraries(mono_vector ${PROJECT_NAME})

add_executable(mono_fourseasons
        Examples/mono/mono_fourseasons.cc)
target_link_libraries(mono_fourseasons ${PROJECT_NAME})

add_executable(mono_oxford
        Examples/mono/mono_oxford.cc)
target_link_libraries(mono_oxford ${PROJECT_NAME})

add_executable(mono_interior
        Examples/mono/mono_interior.cc)
target_link_libraries(mono_interior ${PROJECT_NAME})

