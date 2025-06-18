cmake_minimum_required(VERSION 3.16.2)
project(real_time_3d_tracking VERSION 1.0.0)
set(real_time_3d_tracking ${PROJECT_VERSION})
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")

else()
  set(OPTIMIZATION_FLAGS $ENV{CPP_OPTIMIZATIONS})
  set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -Wno-deprecated-declarations -flto"
  )
endif()

message(STATUS "DEBUG_TIME_LOGGING is ${DEBUG_TIME_LOGGING}")
message(STATUS "DEBUG_SINGLE_IMAGE is ${DEBUG_SINGLE_IMAGE}")

set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cfg)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
)

# ============ Dependencies ============

link_directories(/usr/local/lib)
if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()
set(OPENCV_VERSION $ENV{OPENCV_VERSION})
message(STATUS "OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(cpp_utils REQUIRED)
find_package(detection_inference REQUIRED)
find_package(pose_inference REQUIRED)
find_package(flirmulticamera REQUIRED)
find_package(tensorrt-cpp-api REQUIRED)
find_package(fmt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_interface
  flir_icp_calib
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp ros_node_interface flir_icp_calib
)

set(
  IncludeDirsList
  ${catkin_LIBRARIES} 
  ${SRC_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${tensorrt-cpp-api_INCLUDE_DIRS}
)
include_directories(
  ${catkin_LIBRARIES} 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(
  LibrariesList 
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
  ${catkin_LIBRARIES}
  cpp_utils detection_inference pose_inference flirmulticamera
)

aux_source_directory(src/aabb AABB_SOURCES)
aux_source_directory(src/tracker TRACKER_SOURCES)
aux_source_directory(src/keypoint KPS_SOURCES)
aux_source_directory(src/common COMMON)

set(
  DependenciesList ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

# ============ EXECUTABLES ============
# BENCHMARK
add_executable(inference_benchmark src/nodes/main_benchmark.cpp ${TRACKER_SOURCES} ${AABB_SOURCES} ${KPS_SOURCES} ${COMMON})
add_dependencies(inference_benchmark ${DependenciesList})
target_include_directories(inference_benchmark PUBLIC ${IncludeDirsList})
target_link_libraries(inference_benchmark PUBLIC ${LibrariesList})
set_target_properties(inference_benchmark PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# COLOR
add_executable(interface_color src/nodes/main_interface.cpp ${TRACKER_SOURCES} ${AABB_SOURCES} ${KPS_SOURCES} ${COMMON})
add_dependencies(interface_color ${DependenciesList})
target_include_directories(interface_color PUBLIC ${IncludeDirsList})
target_link_libraries(interface_color PUBLIC ${LibrariesList})
set_target_properties(interface_color PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# KPS133
add_executable(interface_133 src/nodes/main_interface.cpp ${TRACKER_SOURCES} ${AABB_SOURCES} ${KPS_SOURCES} ${COMMON})
add_dependencies(interface_133 ${DependenciesList})
target_include_directories(interface_133 PUBLIC ${IncludeDirsList})
target_link_libraries(interface_133 PUBLIC ${LibrariesList})
set_target_properties(interface_133 PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  
target_compile_definitions(inference_benchmark PUBLIC SINGLE_IMAGE_DEBUG=1)
target_compile_definitions(inference_benchmark PUBLIC TRACK_KPS133=1) 

target_compile_definitions(interface_133 PUBLIC TRACK_KPS133=1)
# target_compile_definitions(interface_133 PUBLIC VIDEO_LOGGING=1)

target_compile_definitions(interface_color PUBLIC TRACK_COLOR=1)
# target_compile_definitions(interface_color PUBLIC VIDEO_LOGGING=1)
