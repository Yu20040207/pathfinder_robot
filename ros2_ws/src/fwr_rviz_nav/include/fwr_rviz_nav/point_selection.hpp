#pragma once

#include <optional>

namespace fwr_rviz_nav
{
struct Point
{
  double x;
  double y;
};

class PointSelection
{
public:
  // One click selects a destination and the robot starts from its current
  // pose. A second click turns the first point into the start and the new
  // point into the goal. A third click begins a new single-point selection.
  void click(double x, double y)
  {
    if (!goal_) {
      goal_ = Point{x, y};
    } else if (!start_) {
      start_ = *goal_;
      goal_ = Point{x, y};
    } else {
      start_.reset();
      goal_ = Point{x, y};
    }
  }

  void clear()
  {
    start_.reset();
    goal_.reset();
  }

  bool has_goal() const { return goal_.has_value(); }
  bool single() const { return goal_.has_value() && !start_.has_value(); }
  bool two_point() const { return start_.has_value() && goal_.has_value(); }
  // Retained for the two-point route builders: ready() means both a selected
  // start and goal exist. Callers that also accept a single destination should
  // use has_goal()/single().
  bool ready() const { return two_point(); }
  const std::optional<Point> & start() const { return start_; }
  const std::optional<Point> & goal() const { return goal_; }

private:
  std::optional<Point> start_;
  std::optional<Point> goal_;
};
}  // namespace fwr_rviz_nav
