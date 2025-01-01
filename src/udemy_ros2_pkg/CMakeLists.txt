cmake_minimum_required(VERSION 3.8)
project(udemy_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Generate custom interface
rosidl_generate_interfaces(
  ${PROJECT_NAME} ## name of project
  "srv/OddEvenCheck.srv" ## interface file list
  "srv/TurnCamera.srv" ## interface file list
  "action/Navigate.action" ## interface file list
  DEPENDENCIES ## just declaration for following pkges
  sensor_msgs geometry_msgs action_msgs
  ADD_LINTER_TESTS ## for adding liter test codes
)

# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(rpm_pub src/rpm_pub.cpp)
ament_target_dependencies(rpm_pub rclcpp std_msgs)

add_executable(speed_calc src/speed_calc.cpp)
ament_target_dependencies(speed_calc rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs)

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp std_msgs)

add_executable(turn_camera_service src/turn_camera_service.cpp)
ament_target_dependencies(turn_camera_service rclcpp std_msgs sensor_msgs OpenCV cv_bridge)

add_executable(turn_camera_client src/turn_camera_client.cpp)
ament_target_dependencies(turn_camera_client rclcpp std_msgs sensor_msgs OpenCV cv_bridge)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp std_msgs geometry_msgs action_msgs rclcpp_action)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp std_msgs geometry_msgs action_msgs rclcpp_action)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(service_server "${cpp_typesupport_target}")
target_link_libraries(service_client "${cpp_typesupport_target}")
target_link_libraries(turn_camera_service "${cpp_typesupport_target}")
target_link_libraries(turn_camera_client "${cpp_typesupport_target}")
target_link_libraries(action_server "${cpp_typesupport_target}")
target_link_libraries(action_client "${cpp_typesupport_target}")

install(TARGETS
  publisher
  subscriber
  rpm_pub
  speed_calc
  service_server
  service_client
  turn_camera_service
  turn_camera_client
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
