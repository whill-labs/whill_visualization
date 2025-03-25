cmake_minimum_required(VERSION 3.8)
project(ros2_whill_visualization_msgs)

# 依存パッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# インターフェース生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolygonArray.msg"
  # 他のメッセージファイルがあれば追加してください
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_package()
