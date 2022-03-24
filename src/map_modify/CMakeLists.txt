cmake_minimum_required(VERSION 3.0.2)
project(map_modify)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_library(jsoncpp_LIB NAMES libjsoncpp.a PATHS vcpkg/packages/jsoncpp_x64-linux/lib REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES map_modify
  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp sensor_msgs std_msgs tf
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/map_modify.cpp)

set(JSON_INC_PATH vcpkg/packages/jsoncpp_x64-osx/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${JSON_INC_PATH})

set(JSON_LIB_PATH vcpkg/packages/jsoncpp_x64-osx/lib)
target_link_directories(${PROJECT_NAME} PUBLIC ${JSON_LIB_PATH})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} 
${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${jsoncpp_LIB}
)