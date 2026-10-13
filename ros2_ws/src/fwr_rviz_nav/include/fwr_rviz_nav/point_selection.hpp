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
  void click(double x, double y)
  {
    if (!start_ || goal_) {
      start_ = Point{x, y};
      goal_.reset();
    } else {
      goal_ = Point{x, y};
    }
  }

  void clear()
  {
    start_.reset();
    goal_.reset();
  }

  bool ready() const { return start_.has_value() && goal_.has_value(); }
  const std::optional<Point> & start() const { return start_; }
  const std::optional<Point> & goal() const { return goal_; }

private:
  std::optional<Point> start_;
  std::optional<Point> goal_;
};
}  // namespace fwr_rviz_nav
