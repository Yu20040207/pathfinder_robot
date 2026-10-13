#pragma once

#include <array>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "fwr_rviz_nav/point_selection.hpp"

namespace fwr_rviz_nav
{

inline std::optional<std::array<nav2_msgs::action::NavigateToPose::Goal, 2>> make_navigation_legs(
  const PointSelection & selection, const builtin_interfaces::msg::Time & stamp)
{
  if (!selection.ready()) {return std::nullopt;}
  std::array<nav2_msgs::action::NavigateToPose::Goal, 2> legs;
  const std::array<Point, 2> selected = {*selection.start(), *selection.goal()};
  for (size_t i = 0; i < legs.size(); ++i) {
    auto & pose = legs[i].pose;
    pose.header.frame_id = "map";
    pose.header.stamp = stamp;
    pose.pose.position.x = selected[i].x;
    pose.pose.position.y = selected[i].y;
    pose.pose.orientation.w = 1.0;
  }
  return legs;
}

}  // namespace fwr_rviz_nav
