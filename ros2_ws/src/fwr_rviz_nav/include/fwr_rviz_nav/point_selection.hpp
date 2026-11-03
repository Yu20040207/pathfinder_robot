#pragma once

#include <optional>
#include <vector>

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
  // 2026-11-03 多点导航：每次点击都追加到路线里。
  //   点 1 次       → 单点导航（从当前位置直接去这个点），旧功能不变
  //   点 2 次及以上 → 第 1 个是起点、最后 1 个是终点、中间是途经点
  //   继续点击       → 继续追加（最后点的那个自动成为终点）
  //   清除选择       → 全部清空
  void click(double x, double y)
  {
    points_.push_back(Point{x, y});
    goal_ = points_.back();
    start_ = points_.size() >= 2 ? std::optional<Point>(points_.front()) : std::nullopt;
  }

  void clear()
  {
    points_.clear();
    start_.reset();
    goal_.reset();
  }

  bool has_goal() const {return !points_.empty();}
  bool single() const {return points_.size() == 1;}
  bool two_point() const {return points_.size() == 2;}
  bool multi() const {return points_.size() >= 2;}
  // 兼容旧调用：ready() 表示"起点和终点都齐了"。
  bool ready() const {return multi();}
  std::size_t size() const {return points_.size();}
  const std::vector<Point> & points() const {return points_;}
  const std::optional<Point> & start() const {return start_;}
  const std::optional<Point> & goal() const {return goal_;}

private:
  std::vector<Point> points_;
  std::optional<Point> start_;
  std::optional<Point> goal_;
};
}  // namespace fwr_rviz_nav
