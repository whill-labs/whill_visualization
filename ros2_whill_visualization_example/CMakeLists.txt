cmake_minimum_required(VERSION 3.8)
project(ros2_whill_visualization_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_whill_visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# デモ用パブリッシャーの実行ファイル
add_executable(polygon_array_publisher
  src/polygon_array_publisher.cpp
)

ament_target_dependencies(polygon_array_publisher
  rclcpp
  std_msgs
  geometry_msgs
  ros2_whill_visualization_msgs
)

# インストール
install(
  TARGETS polygon_array_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# launchファイルのインストール
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
