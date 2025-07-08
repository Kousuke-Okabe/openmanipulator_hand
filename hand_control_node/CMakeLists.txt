cmake_minimum_required(VERSION 3.8)
project(hand_control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hand_control_interfaces REQUIRED)

include_directories(include)

add_executable(hand_control_node src/hand_current-based_position.cpp)
target_link_libraries(hand_control_node PUBLIC
  dynamixel_sdk::dynamixel_sdk
  ${hand_control_interfaces_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS
  hand_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
