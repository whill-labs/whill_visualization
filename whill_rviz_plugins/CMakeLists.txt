cmake_minimum_required(VERSION 3.8)
project(whill_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(whill_visualization_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)

# インクルードディレクトリ
include_directories(include)

# QT設定
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# プラグインライブラリ
add_library(${PROJECT_NAME} SHARED
  src/polygon_array_display.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  whill_visualization_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY"
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# インストール
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# エクスポート
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  whill_visualization_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
