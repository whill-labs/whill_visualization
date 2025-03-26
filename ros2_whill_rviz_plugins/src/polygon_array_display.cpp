#include "polygon_array_display.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <QColor>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>

namespace ros2_whill_visualization
{

PolygonArrayDisplay::PolygonArrayDisplay()
: only_border_(true), enable_lighting_(true), show_normal_(true), normal_length_(0.1)
{
  // Initialize properties
  coloring_property_ = new rviz_common::properties::EnumProperty(
    "Coloring", "Auto", "Coloring method for polygons", this, SLOT(updateColoring()));
  coloring_property_->addOption("Auto", 0);
  coloring_property_->addOption("Flat color", 1);
  coloring_property_->addOption("Likelihood", 2);
  coloring_property_->addOption("Label", 3);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 255, 0), "Color to draw the polygons when using flat coloring.", this,
    SLOT(queueRender()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Amount of transparency to apply to the polygon.", this, SLOT(queueRender()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  only_border_property_ = new rviz_common::properties::BoolProperty(
    "Only border", true, "Only shows the borders of polygons", this, SLOT(updateOnlyBorder()));

  show_normal_property_ = new rviz_common::properties::BoolProperty(
    "Show normal", true, "Show normal direction of polygons", this, SLOT(updateShowNormal()));

  enable_lighting_property_ = new rviz_common::properties::BoolProperty(
    "Enable lighting", true, "Enable lighting for filled polygons", this,
    SLOT(updateEnableLighting()));

  normal_length_property_ = new rviz_common::properties::FloatProperty(
    "Normal length", 0.1, "Length of the normal arrow", this, SLOT(updateNormalLength()));
  normal_length_property_->setMin(0.0);
}

PolygonArrayDisplay::~PolygonArrayDisplay()
{
  // Cleanup
  for (size_t i = 0; i < lines_.size(); i++) {
    delete lines_[i];
  }

  for (size_t i = 0; i < materials_.size(); i++) {
    materials_[i]->unload();
    Ogre::MaterialManager::getSingleton().remove(materials_[i]->getName());
  }

  for (size_t i = 0; i < manual_objects_.size(); i++) {
    scene_manager_->destroyManualObject(manual_objects_[i]);
    scene_manager_->destroySceneNode(scene_nodes_[i]);
  }

  for (size_t i = 0; i < arrow_nodes_.size(); i++) {
    scene_manager_->destroySceneNode(arrow_nodes_[i]);
  }
}

void PolygonArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateOnlyBorder();
  updateColoring();
  updateShowNormal();
  updateNormalLength();
  updateEnableLighting();
}

void PolygonArrayDisplay::reset()
{
  MFDClass::reset();
  for (size_t i = 0; i < manual_objects_.size(); i++) {
    manual_objects_[i]->clear();
  }
}

void PolygonArrayDisplay::updateSceneNodes(
  const ros2_whill_visualization_msgs::msg::PolygonArray::ConstSharedPtr & msg)
{
  // シーンノードの更新
  int scale_factor = 2;
  if (only_border_) {
    scale_factor = 1;
  }

  // Manual objectsの作成・更新
  if (msg->polygons.size() * scale_factor > manual_objects_.size()) {
    for (size_t i = manual_objects_.size(); i < msg->polygons.size() * scale_factor; i++) {
      Ogre::SceneNode * scene_node = scene_node_->createChildSceneNode();
      Ogre::ManualObject * manual_object = scene_manager_->createManualObject();
      manual_object->setDynamic(true);
      scene_node->attachObject(manual_object);
      manual_objects_.push_back(manual_object);
      scene_nodes_.push_back(scene_node);
    }
  } else if (msg->polygons.size() * scale_factor < manual_objects_.size()) {
    for (size_t i = msg->polygons.size() * scale_factor; i < manual_objects_.size(); i++) {
      manual_objects_[i]->setVisible(false);
    }
  }

  // Arrow objectsの作成・更新
  if (msg->polygons.size() > arrow_objects_.size()) {
    for (size_t i = arrow_objects_.size(); i < msg->polygons.size(); i++) {
      Ogre::SceneNode * scene_node = scene_node_->createChildSceneNode();
      ArrowPtr arrow(new Arrow(scene_manager_, scene_node));
      scene_node->setVisible(false);
      arrow_objects_.push_back(arrow);
      arrow_nodes_.push_back(scene_node);
    }
  } else if (msg->polygons.size() < arrow_nodes_.size()) {
    for (size_t i = msg->polygons.size(); i < arrow_nodes_.size(); i++) {
      arrow_nodes_[i]->setVisible(false);
    }
  }
}

void PolygonArrayDisplay::allocateMaterials(int num)
{
  if (only_border_) {
    return;
  }

  static uint32_t count = 0;

  if (num > static_cast<int>(materials_.size())) {
    for (size_t i = materials_.size(); static_cast<int>(i) < num; i++) {
      std::stringstream ss;
      ss << "PolygonArrayMaterial" << count++;
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(enable_lighting_);
      material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
      materials_.push_back(material);
    }
  }
}

void PolygonArrayDisplay::updateLines(int num)
{
  if (num > static_cast<int>(lines_.size())) {
    for (size_t i = lines_.size(); i < static_cast<size_t>(num); i++) {
      rviz_rendering::BillboardLine * line =
        new rviz_rendering::BillboardLine(scene_manager_, scene_nodes_[i]);
      line->setLineWidth(0.01);
      line->setNumLines(1);
      lines_.push_back(line);
    }
  }

  for (size_t i = 0; i < lines_.size(); i++) {
    lines_[i]->clear();
  }
}

Ogre::ColourValue PolygonArrayDisplay::getColor(size_t index)
{
  Ogre::ColourValue color;

  if (coloring_method_ == "auto") {
    // Auto coloring - use index based coloring
    const float hue = static_cast<float>(index % 20) / 20.0f;
    const QColor qcolor = QColor::fromHsvF(hue, 1.0, 1.0);
    color = Ogre::ColourValue(qcolor.redF(), qcolor.greenF(), qcolor.blueF(), 1.0f);
  } else if (coloring_method_ == "flat") {
    // Flat coloring - use user defined color
    color = rviz_common::properties::qtToOgre(color_property_->getColor());
  } else if (coloring_method_ == "likelihood") {
    // Likelihood coloring
    if (latest_msg_->likelihood.empty() || latest_msg_->likelihood.size() <= index) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message does not have likelihood fields");
      color = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);
    } else {
      // Heat map coloring (red to green)
      float likelihood = latest_msg_->likelihood[index];
      color = Ogre::ColourValue(1.0 - likelihood, likelihood, 0.0, 1.0);
    }
  } else if (coloring_method_ == "label") {
    // Label coloring
    if (latest_msg_->labels.empty() || latest_msg_->labels.size() <= index) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message does not have labels fields");
      color = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);
    } else {
      // Label based coloring
      uint32_t label = latest_msg_->labels[index];
      const float hue = static_cast<float>(label % 20) / 20.0f;
      const QColor qcolor = QColor::fromHsvF(hue, 1.0, 1.0);
      color = Ogre::ColourValue(qcolor.redF(), qcolor.greenF(), qcolor.blueF(), 1.0f);
    }
  }

  // Apply alpha
  color.a = alpha_property_->getFloat();
  return color;
}

void PolygonArrayDisplay::processLine(
  const size_t i, const geometry_msgs::msg::PolygonStamped & polygon)
{
  Ogre::SceneNode * scene_node = scene_nodes_[i];
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (!getTransform(polygon.header, position, orientation)) {
    return;
  }

  scene_node->setPosition(position);
  scene_node->setOrientation(orientation);

  rviz_rendering::BillboardLine * line = lines_[i];
  line->clear();
  line->setMaxPointsPerLine(polygon.polygon.points.size() + 1);

  Ogre::ColourValue color = getColor(i);
  line->setColor(color.r, color.g, color.b, color.a);

  // ポリゴンの頂点を追加
  for (size_t j = 0; j < polygon.polygon.points.size(); ++j) {
    Ogre::Vector3 step_position;
    step_position.x = polygon.polygon.points[j].x;
    step_position.y = polygon.polygon.points[j].y;
    step_position.z = polygon.polygon.points[j].z;
    line->addPoint(step_position);
  }

  // 最初の点に戻って閉じる
  if (!polygon.polygon.points.empty()) {
    Ogre::Vector3 step_position;
    step_position.x = polygon.polygon.points[0].x;
    step_position.y = polygon.polygon.points[0].y;
    step_position.z = polygon.polygon.points[0].z;
    line->addPoint(step_position);
  }
}

void PolygonArrayDisplay::processPolygonMaterial(const size_t i)
{
  Ogre::ColourValue color = getColor(i);
  materials_[i]->getTechnique(0)->setLightingEnabled(enable_lighting_);
  materials_[i]->getTechnique(0)->setAmbient(color * 0.5);
  materials_[i]->getTechnique(0)->setDiffuse(color);

  if (color.a < 0.9998) {
    materials_[i]->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    materials_[i]->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    materials_[i]->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    materials_[i]->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

void PolygonArrayDisplay::processPolygon(
  const size_t i, const geometry_msgs::msg::PolygonStamped & polygon)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (!getTransform(polygon.header, position, orientation)) {
    return;
  }

  Ogre::SceneNode * scene_node = scene_nodes_[i * 2];
  Ogre::ManualObject * manual_object = manual_objects_[i * 2];
  Ogre::ColourValue color = getColor(i);

  scene_node->setPosition(position);
  scene_node->setOrientation(orientation);
  manual_object->clear();
  manual_object->setVisible(true);

  // ポリゴンの三角形分割と描画
  if (polygon.polygon.points.size() < 3) {
    return;  // 3点未満のポリゴンは描画できない
  }

  // 多角形を描画（単純な三角形扇）
  manual_object->begin(materials_[i]->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN);

  // 重心を計算して最初の点として追加
  Ogre::Vector3 centroid(0, 0, 0);
  for (size_t j = 0; j < polygon.polygon.points.size(); j++) {
    centroid.x += polygon.polygon.points[j].x;
    centroid.y += polygon.polygon.points[j].y;
    centroid.z += polygon.polygon.points[j].z;
  }
  centroid /= static_cast<float>(polygon.polygon.points.size());

  manual_object->position(centroid);
  manual_object->colour(color.r, color.g, color.b, color.a);

  // 各頂点を追加
  for (size_t j = 0; j < polygon.polygon.points.size(); j++) {
    manual_object->position(
      polygon.polygon.points[j].x, polygon.polygon.points[j].y, polygon.polygon.points[j].z);
    manual_object->colour(color.r, color.g, color.b, color.a);
  }

  // 最初の頂点を再度追加して閉じる
  if (polygon.polygon.points.size() > 0) {
    manual_object->position(
      polygon.polygon.points[0].x, polygon.polygon.points[0].y, polygon.polygon.points[0].z);
    manual_object->colour(color.r, color.g, color.b, color.a);
  }

  manual_object->end();
}

void PolygonArrayDisplay::processNormal(
  const size_t i, const geometry_msgs::msg::PolygonStamped & polygon)
{
  if (polygon.polygon.points.size() < 3) {
    return;  // 法線を計算するには少なくとも3点必要
  }

  Ogre::SceneNode * scene_node = arrow_nodes_[i];
  scene_node->setVisible(true);
  ArrowPtr arrow = arrow_objects_[i];

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (!getTransform(polygon.header, position, orientation)) {
    return;
  }

  scene_node->setPosition(position);
  scene_node->setOrientation(orientation);

  // 重心を計算
  Ogre::Vector3 centroid(0, 0, 0);
  for (size_t j = 0; j < polygon.polygon.points.size(); j++) {
    centroid.x += polygon.polygon.points[j].x;
    centroid.y += polygon.polygon.points[j].y;
    centroid.z += polygon.polygon.points[j].z;
  }
  centroid /= static_cast<float>(polygon.polygon.points.size());

  // 法線を計算（最初の3点を使用）
  Ogre::Vector3 v1(
    polygon.polygon.points[1].x - polygon.polygon.points[0].x,
    polygon.polygon.points[1].y - polygon.polygon.points[0].y,
    polygon.polygon.points[1].z - polygon.polygon.points[0].z);

  Ogre::Vector3 v2(
    polygon.polygon.points[2].x - polygon.polygon.points[0].x,
    polygon.polygon.points[2].y - polygon.polygon.points[0].y,
    polygon.polygon.points[2].z - polygon.polygon.points[0].z);

  Ogre::Vector3 normal = v1.crossProduct(v2);
  normal.normalise();

  if (std::isnan(normal.x) || std::isnan(normal.y) || std::isnan(normal.z)) {
    RCLCPP_ERROR(rclcpp::get_logger("polygon_array_display"), "Failed to compute normal direction");
    Ogre::Vector3 zeroscale(0, 0, 0);
    arrow->setScale(zeroscale);
    return;
  }

  Ogre::Vector3 scale(normal_length_, normal_length_, normal_length_);
  arrow->setPosition(centroid);
  arrow->setDirection(normal);
  arrow->setScale(scale);
  arrow->setColor(getColor(i).r, getColor(i).g, getColor(i).b, getColor(i).a);
}

void PolygonArrayDisplay::processMessage(
  ros2_whill_visualization_msgs::msg::PolygonArray::ConstSharedPtr msg)
{
  // Process message
  // Check if each polygon is valid - modify validateFloats function
  bool valid = true;
  for (const auto & polygon : msg->polygons) {
    for (const auto & point : polygon.polygon.points) {
      if (
        !rviz_common::validateFloats(point.x) || !rviz_common::validateFloats(point.y) ||
        !rviz_common::validateFloats(point.z)) {
        valid = false;
        break;
      }
    }
    if (!valid) {
      break;
    }
  }

  // likelihoodもチェック
  if (valid) {
    for (const auto & val : msg->likelihood) {
      if (!rviz_common::validateFloats(val)) {
        valid = false;
        break;
      }
    }
  }

  if (!valid) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (NaNs or infs)");
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");

  latest_msg_ = msg;

  // シーンノードとマテリアルの更新
  updateSceneNodes(msg);
  allocateMaterials(msg->polygons.size());
  updateLines(msg->polygons.size());

  if (only_border_) {
    // Display only borders
    for (size_t i = 0; i < manual_objects_.size(); i++) {
      manual_objects_[i]->setVisible(false);
    }

    for (size_t i = 0; i < msg->polygons.size(); i++) {
      const auto & polygon = msg->polygons[i];
      if (polygon.polygon.points.size() >= 3) {
        processLine(i, polygon);
      }
    }
  } else {
    // Fill display
    for (size_t i = 0; i < msg->polygons.size(); i++) {
      processPolygonMaterial(i);
    }

    for (size_t i = 0; i < msg->polygons.size(); i++) {
      const auto & polygon = msg->polygons[i];
      processPolygon(i, polygon);
    }
  }

  // Normal display
  if (show_normal_) {
    for (size_t i = 0; i < msg->polygons.size(); i++) {
      const auto & polygon = msg->polygons[i];
      if (polygon.polygon.points.size() >= 3) {
        processNormal(i, polygon);
      }
    }
  }
}

bool PolygonArrayDisplay::getTransform(
  const std_msgs::msg::Header & header, Ogre::Vector3 & position, Ogre::Quaternion & orientation)
{
  const auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(header, position, orientation)) {
    std::string error;
    error = "Failed to transform from frame [" + header.frame_id + "] to frame [" +
            fixed_frame_.toStdString() + "]";
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(error));
    return false;
  }
  return true;
}

void PolygonArrayDisplay::updateColoring()
{
  switch (coloring_property_->getOptionInt()) {
    case 0:
      coloring_method_ = "auto";
      color_property_->hide();
      break;
    case 1:
      coloring_method_ = "flat";
      color_property_->show();
      break;
    case 2:
      coloring_method_ = "likelihood";
      color_property_->hide();
      break;
    case 3:
      coloring_method_ = "label";
      color_property_->hide();
      break;
    default:
      coloring_method_ = "auto";
      color_property_->hide();
      break;
  }
}

void PolygonArrayDisplay::updateOnlyBorder() { only_border_ = only_border_property_->getBool(); }

void PolygonArrayDisplay::updateShowNormal()
{
  show_normal_ = show_normal_property_->getBool();
  if (show_normal_) {
    normal_length_property_->show();
  } else {
    normal_length_property_->hide();
    for (size_t i = 0; i < arrow_nodes_.size(); i++) {
      arrow_nodes_[i]->setVisible(false);
    }
  }
}

void PolygonArrayDisplay::updateEnableLighting()
{
  enable_lighting_ = enable_lighting_property_->getBool();
}

void PolygonArrayDisplay::updateNormalLength()
{
  normal_length_ = normal_length_property_->getFloat();
}

}  // namespace ros2_whill_visualization

#include <pluginlib/class_list_macros.hpp>

#include "moc_polygon_array_display.cpp"
PLUGINLIB_EXPORT_CLASS(ros2_whill_visualization::PolygonArrayDisplay, rviz_common::Display)
