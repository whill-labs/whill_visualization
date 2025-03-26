# ros2_whill_visualization

## Overview

ros2_whill_visualization is a collection of visualization-related packages for use in ROS2 environments. It consists of the following three packages:

* ros2_whill_visualization_msgs
  * Provides message definitions like PolygonArray
  * References the Polygon Array msg from [ROS1's jsk_recognition](https://github.com/jsk-ros-pkg/jsk_recognition) ported to ROS2
* ros2_whill_visualization_rviz_plugins
  * Provides plugins for displaying PolygonArray and other messages in Rviz2
  * References the Polygon Array Rviz Plugin from [ROS1's jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization) ported to ROS2
* ros2_whill_visualization_example
  * Provides usage examples for messages and plugins
  * Includes demo nodes and launch files

## Installation

```bash
# Change to workspace directory
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/whill/ros2_whill_visualization.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
```

## Developer Information

- Copyright (c) 2023, WHILL Inc.

## License

This package is provided under the MIT License.

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
