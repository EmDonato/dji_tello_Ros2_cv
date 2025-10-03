cmake_minimum_required(VERSION 3.8)
project(dji_tello_aruco)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Trova dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dji_tello_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)               # per Quaternion

# Eseguibile
add_executable(tello_aruco_node src/aruco_node.cpp)

# Collega librerie
ament_target_dependencies(tello_aruco_node
  rclcpp
  sensor_msgs
  cv_bridge
  dji_tello_interfaces
  tf2_ros
)

target_link_libraries(tello_aruco_node
  ${OpenCV_LIBS}
)

# Installa
install(TARGETS tello_aruco_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

