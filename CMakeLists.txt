cmake_minimum_required(VERSION 2.8.3)

project(ladybug_slam)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

set(DEPRECATION_FLAG "-Wno-deprecated-declarations")
set(REORDER_FLAG "-Wno-reorder")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG} ${REORDER_FLAG} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam2/cmake_modules)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ladybug_slam_lib
  octomap_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  roslaunch
  image_transport
  ladybug_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

find_package(octomap REQUIRED)

include_directories(
${OCTOMAP_INCLUDE_DIRS}
${roscpp_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${octomap_ros_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/../ladybug_slam_lib
${PROJECT_SOURCE_DIR}/../ladybug_slam_lib/include
${PROJECT_SOURCE_DIR}/../ladybug_slam_lib/include/elas
)

add_library(PublisherUtils
 ${PROJECT_SOURCE_DIR}/include/PublisherUtils.h
 ${PROJECT_SOURCE_DIR}/src/PublisherUtils.cpp
 ${PROJECT_SOURCE_DIR}/include/PublisherUtils_impl.h
)

add_library(ScaleCorrector
 ${PROJECT_SOURCE_DIR}/include/ScaleCorrector.h
 ${PROJECT_SOURCE_DIR}/src/ScaleCorrector.cpp
)

set(LIBS
  ${OCTOMAP_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${octomap_ros_LIBRARIES}
  ladybug_slam_lib
  PublisherUtils
  ScaleCorrector
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../ladybug_slam_lib/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../ladybug_slam_lib/Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../ladybug_slam_lib/Thirdparty/OpenABLE/libOpenABLE.so
  ${PCL_LIBRARIES}
)

roslaunch_add_file_check(launch)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES retrorov_model_bathymetric
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime geometry_msgs ladybug_msgs message_runtime 
  octomap_ros ladybug_slam_lib
#  DEPENDS system_lib
)

add_executable(slam src/slam.cpp src/ROSPublisher.cpp)
add_dependencies(slam ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(slam ${LIBS})


add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})