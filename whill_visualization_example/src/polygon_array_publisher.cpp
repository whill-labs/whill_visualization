#include <chrono>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "whill_visualization_msgs/msg/polygon_array.hpp"

using namespace std::chrono_literals;
using namespace whill_visualization_msgs::msg;

class PolygonArrayPublisher : public rclcpp::Node
{
public:
  PolygonArrayPublisher() : Node("polygon_array_publisher")
  {
    publisher_ = this->create_publisher<PolygonArray>("polygon_array", 10);
    timer_ =
      this->create_wall_timer(100ms, std::bind(&PolygonArrayPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto now = this->now();
    PolygonArray message;
    message.header.stamp = now;
    message.header.frame_id = "map";

    for (int i = 0; i < 4; ++i) {
      geometry_msgs::msg::PolygonStamped polygon;
      polygon.header.stamp = now;
      polygon.header.frame_id = "map";

      double radius = 1.0;
      double offset_x = i * 2.5;
      double offset_y = 0.0;

      // Square
      if (i == 0) {
        add_vertex(polygon, offset_x - radius, offset_y - radius, 0.0);
        add_vertex(polygon, offset_x + radius, offset_y - radius, 0.0);
        add_vertex(polygon, offset_x + radius, offset_y + radius, 0.0);
        add_vertex(polygon, offset_x - radius, offset_y + radius, 0.0);
      }
      // Triangle
      else if (i == 1) {
        add_vertex(polygon, offset_x, offset_y + radius, 0.0);
        add_vertex(polygon, offset_x - radius, offset_y - radius, 0.0);
        add_vertex(polygon, offset_x + radius, offset_y - radius, 0.0);
      }
      // Pentagon
      else if (i == 2) {
        for (int j = 0; j < 5; ++j) {
          double angle = 2.0 * M_PI * j / 5.0;
          add_vertex(
            polygon, offset_x + radius * std::cos(angle), offset_y + radius * std::sin(angle), 0.0);
        }
      }
      // Hexagon
      else {
        for (int j = 0; j < 6; ++j) {
          double angle = 2.0 * M_PI * j / 6.0;
          add_vertex(
            polygon, offset_x + radius * std::cos(angle), offset_y + radius * std::sin(angle), 0.0);
        }
      }

      message.polygons.push_back(polygon);
      message.labels.push_back(i);
      message.likelihood.push_back(static_cast<float>(i) / 3.0);
    }

    publisher_->publish(message);
    RCLCPP_INFO(
      this->get_logger(), "PolygonArray published with %zu polygons", message.polygons.size());
  }

  void add_vertex(geometry_msgs::msg::PolygonStamped & polygon, double x, double y, double z)
  {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(z);
    polygon.polygon.points.push_back(point);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PolygonArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolygonArrayPublisher>());
  rclcpp::shutdown();
  return 0;
}
