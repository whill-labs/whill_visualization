# ros2_whill_visualization

## 概要

ros2_whill_visualizationは、ROS2環境で使用される可視化関連のパッケージ群です。以下の3つのパッケージで構成されています：

* ros2_whill_visualization_msgs
  * PolygonArray等のメッセージ定義を提供
* ros2_whill_visualization_rviz_plugins
  * PolygonArray等をRviz2で表示するためのプラグインを提供
* ros2_whill_visualization_example
  * メッセージとプラグインの使用例を提供
  * デモ用のノードとlaunchファイルを含む

## インストール方法

```bash
# ワークスペースに移動
cd ~/ros2_ws/src

# リポジトリをクローン
git clone https://github.com/whill/ros2_whill_visualization.git

# 依存関係をインストール
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --packages-select ros2_whill_visualization
```

## 開発者情報

- Copyright (c) 2023, WHILL Inc.

## ライセンス

このパッケージはMITライセンスの下で提供されています。

MIT License

Copyright (c) 2023 WHILL Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
