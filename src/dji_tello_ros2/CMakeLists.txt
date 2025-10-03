cmake_minimum_required(VERSION 3.8)
project(dji_tello_ros2)

# --- Dipendenze ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)          # per Odometry
find_package(tf2 REQUIRED)               # per Quaternion
find_package(tf2_ros REQUIRED)               # per Quaternion
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)         # <-- aggiunto
find_package(dji_tello_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)

# --- Libreria comune (per la comunicazione col Tello) ---
add_library(tello_lib
  src/telloComunication.cpp
  src/telloInterface.cpp
)

target_include_directories(tello_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(tello_lib
  ${OpenCV_LIBS}
)

ament_target_dependencies(tello_lib
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  dji_tello_interfaces
  camera_info_manager 
  tf2_ros
)

# --- Nodo driver ---
add_executable(tello_driver_node src/tello_driver_node.cpp)
target_link_libraries(tello_driver_node
  tello_lib
)
ament_target_dependencies(tello_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  cv_bridge          # <-- aggiunto
  dji_tello_interfaces
)

# --- Nodo gamepad ---
add_executable(gamepad_node src/gamepad_node.cpp)
target_link_libraries(gamepad_node
  tello_lib
)
ament_target_dependencies(gamepad_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs           # solo se usi odometry
  tf2                # solo se usi quaternion
  dji_tello_interfaces
)

# --- Install launch files ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# --- Installazione ---
install(TARGETS
  tello_lib
  tello_driver_node
  gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


