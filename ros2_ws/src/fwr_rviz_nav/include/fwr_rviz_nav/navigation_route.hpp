#pragma once

#include <array>
#include <cmath>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>

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

inline std::optional<std::array<nav2_msgs::action::ComputePathToPose::Goal, 2>>
make_real_preview_legs(
  const PointSelection & selection, const builtin_interfaces::msg::Time & stamp)
{
  if (!selection.ready()) {return std::nullopt;}
  std::array<nav2_msgs::action::ComputePathToPose::Goal, 2> legs;
  auto fill = [&](geometry_msgs::msg::PoseStamped & pose, const Point & point) {
    pose.header.frame_id = "map";
    pose.header.stamp = stamp;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.w = 1.0;
  };
  fill(legs[0].goal, *selection.start());
  legs[0].use_start = false;
  fill(legs[1].start, *selection.start());
  fill(legs[1].goal, *selection.goal());
  legs[1].use_start = true;
  return legs;
}

inline std::optional<nav2_msgs::action::ComputePathToPose::Goal> make_single_preview_goal(
  const PointSelection & selection, const builtin_interfaces::msg::Time & stamp)
{
  if (!selection.single()) {return std::nullopt;}
  nav2_msgs::action::ComputePathToPose::Goal goal;
  goal.goal.header.frame_id = "map";
  goal.goal.header.stamp = stamp;
  goal.goal.pose.position.x = selection.goal()->x;
  goal.goal.pose.position.y = selection.goal()->y;
  goal.goal.pose.orientation.w = 1.0;
  goal.use_start = false;
  return goal;
}

inline std::optional<nav2_msgs::action::NavigateToPose::Goal> make_single_navigation_goal(
  const PointSelection & selection, const builtin_interfaces::msg::Time & stamp)
{
  if (!selection.single()) {return std::nullopt;}
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = stamp;
  goal.pose.pose.position.x = selection.goal()->x;
  goal.pose.pose.position.y = selection.goal()->y;
  goal.pose.pose.orientation.w = 1.0;
  return goal;
}

inline double path_length_m(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

inline bool real_preview_legs_planned(double first_length_m, double second_length_m)
{
  return std::isfinite(first_length_m) && first_length_m >= 0.0 &&
         std::isfinite(second_length_m) && second_length_m >= 0.0;
}

}  // namespace fwr_rviz_nav
