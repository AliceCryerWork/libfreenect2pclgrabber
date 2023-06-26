# Copyright 2015, Giacomo Dabisias"
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# @Author 
# Giacomo Dabisias, PhD Student
# PERCRO, (Laboratory of Perceptual Robotics)
# Scuola Superiore Sant'Anna
# via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy

cmake_minimum_required(VERSION 3.5)
Project(kinect2grabber)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WITH_OPENCL "adds opencl support for freenect2" OFF)
if(${WITH_OPENCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_OPENCL")
endif()

option(WITH_CUDA "adds cuda support for freenect2" OFF)
if(${WITH_CUDA})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CUDA")
endif()

option(WITH_SERIALIZATION "adds support for boost serialization" OFF)
if(${WITH_SERIALIZATION})
	find_package(Boost REQUIRED)
	include_directories(${INCLUDE})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_SERIALIZATION")
	set(BOOST_LIBS "boost_serialization")
endif()
include_directories(/opt/ros/humble/include)

	find_package(ament_cmake REQUIRED)
	find_package(argparse REQUIRED)
	find_package(freenect2 REQUIRED)
	find_package(OpenCV REQUIRED)
	find_package(rclcpp REQUIRED)
	find_package(std_msgs REQUIRED)
	find_package(sensor_msgs REQUIRED)
	find_package(pcl_conversions REQUIRED)
	find_package(PCL 1.10 REQUIRED)
 	find_package(eigen3_cmake_module REQUIRED)
	find_package(Eigen3 REQUIRED)

install(DIRECTORY "include" DESTINATION ".")


add_library(kinect2grabber
  "multi_test.cpp"
  "test_ros.cpp"
  "test.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${Eigen3_INCLUDE_DIRS})

target_link_libraries( ${PROJECT_NAME}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  argparse freenect2 OpenCV rclcpp std_msgs sensor_msgs pcl_conversions)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

set(EXECUTABLES
"test_ros.cpp")

foreach(EXECUTABLE ${EXECUTABLES})
  get_filename_component(TARGET ${EXECUTABLE} NAME_WE)

  add_executable(${TARGET} ${EXECUTABLE})
  target_link_libraries(${TARGET} ${PROJECT_NAME})

  install(TARGETS ${TARGET}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

ament_export_dependencies(argparse freenect2 OpenCV rclcpp sensor_msgs std_msgs pcl_conversions)

ament_package()
