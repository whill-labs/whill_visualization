// Copyright (c) 2023, WHILL Inc.
// All rights reserved.

#ifndef POLYGON_ARRAY_DISPLAY_HPP_
#define POLYGON_ARRAY_DISPLAY_HPP_

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <string>
#include <vector>

#include "whill_visualization_msgs/msg/polygon_array.hpp"

// Add validateFloats function for custom message types
namespace rviz_common
{
bool validateFloats(const geometry_msgs::msg::PolygonStamped & polygon);

inline bool validateFloats(const geometry_msgs::msg::PolygonStamped & polygon)
{
  return validateFloats(polygon.polygon.points);
}

bool validateFloats(const whill_visualization_msgs::msg::PolygonArray & msg);

inline bool validateFloats(const whill_visualization_msgs::msg::PolygonArray & msg)
{
  for (const auto & polygon : msg.polygons) {
    if (!validateFloats(polygon)) {
      return false;
    }
  }

  for (const auto & val : msg.likelihood) {
    if (!validateFloats(val)) {
      return false;
    }
  }

  return true;
}
}  // namespace rviz_common

namespace whill_visualization
{
class PolygonArrayDisplay
: public rviz_common::MessageFilterDisplay<whill_visualization_msgs::msg::PolygonArray>
{
  Q_OBJECT

public:
  using Arrow = rviz_rendering::Arrow;
  using ArrowPtr = std::shared_ptr<Arrow>;

  PolygonArrayDisplay();
  virtual ~PolygonArrayDisplay();

protected:
  virtual void onInitialize() override;
  virtual void reset() override;
  virtual void processMessage(
    whill_visualization_msgs::msg::PolygonArray::ConstSharedPtr msg) override;

private:
  void updateSceneNodes(const whill_visualization_msgs::msg::PolygonArray::ConstSharedPtr & msg);
  void allocateMaterials(int num);
  void updateLines(int num);
  Ogre::ColourValue getColor(size_t index);
  void processLine(const size_t i, const geometry_msgs::msg::PolygonStamped & polygon);
  void processPolygon(const size_t i, const geometry_msgs::msg::PolygonStamped & polygon);
  void processNormal(const size_t i, const geometry_msgs::msg::PolygonStamped & polygon);
  void processPolygonMaterial(const size_t i);
  bool getTransform(
    const std_msgs::msg::Header & header, Ogre::Vector3 & position, Ogre::Quaternion & orientation);

  // Properties
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::BoolProperty * only_border_property_;
  rviz_common::properties::EnumProperty * coloring_property_;
  rviz_common::properties::BoolProperty * show_normal_property_;
  rviz_common::properties::BoolProperty * enable_lighting_property_;
  rviz_common::properties::FloatProperty * normal_length_property_;

  // State variables
  bool only_border_;
  bool enable_lighting_;
  std::string coloring_method_;
  bool show_normal_;
  double normal_length_;
  whill_visualization_msgs::msg::PolygonArray::ConstSharedPtr latest_msg_;

  // Rendering objects
  std::vector<Ogre::ManualObject *> manual_objects_;
  std::vector<Ogre::SceneNode *> scene_nodes_;
  std::vector<Ogre::SceneNode *> arrow_nodes_;
  std::vector<ArrowPtr> arrow_objects_;
  std::vector<Ogre::MaterialPtr> materials_;
  std::vector<rviz_rendering::BillboardLine *> lines_;

private Q_SLOTS:
  void updateColoring();
  void updateOnlyBorder();
  void updateShowNormal();
  void updateEnableLighting();
  void updateNormalLength();
};

}  // namespace whill_visualization

#endif  // POLYGON_ARRAY_DISPLAY_HPP_
