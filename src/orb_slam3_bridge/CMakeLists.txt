cmake_minimum_required(VERSION 3.16)
project(orb_slam3_bridge LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-deprecated-declarations -Wno-unused-parameter)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marker_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin QUIET)

# ORB_SLAM3_ROOT is provided by scripts/install_orb_slam3.sh and is a
# directory containing the ORB-SLAM3 source tree (with built libraries
# in $ORB_SLAM3_ROOT/lib and Thirdparty libs already compiled).
if(DEFINED ENV{ORB_SLAM3_ROOT})
  set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}")
endif()

if(NOT ORB_SLAM3_ROOT)
  message(WARNING
    "ORB_SLAM3_ROOT is not set - skipping orb_slam3_node compilation. "
    "Run scripts/install_orb_slam3.sh and re-source the shell, then re-build.")
  ament_package()
  return()
endif()

if(NOT EXISTS "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
  message(WARNING
    "ORB_SLAM3_ROOT=${ORB_SLAM3_ROOT} but libORB_SLAM3.so is missing - "
    "skipping orb_slam3_node compilation.")
  ament_package()
  return()
endif()

set(ORB_SLAM3_INCLUDE_DIRS
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
)
set(ORB_SLAM3_LIBS
  "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so"
)

add_executable(orb_slam3_node src/orb_slam3_node.cpp)
target_include_directories(orb_slam3_node PRIVATE
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
if(Pangolin_FOUND)
  target_include_directories(orb_slam3_node PRIVATE ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(orb_slam3_node ${Pangolin_LIBRARIES})
endif()
target_link_libraries(orb_slam3_node
  ${OpenCV_LIBS}
  ${ORB_SLAM3_LIBS}
)
ament_target_dependencies(orb_slam3_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs marker_interfaces
)

# RPATH so the executable finds the libraries shipped with ORB-SLAM3
# both at build-time and after `ros2 run`.
set_target_properties(orb_slam3_node PROPERTIES
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS orb_slam3_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
