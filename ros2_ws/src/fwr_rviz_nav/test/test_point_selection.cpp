#include <gtest/gtest.h>
#include "fwr_rviz_nav/point_selection.hpp"

TEST(PointSelection, FirstClickIsSingleGoalThenSecondClickCreatesTwoPoint)
{
  fwr_rviz_nav::PointSelection selection;
  EXPECT_FALSE(selection.has_goal());
  EXPECT_FALSE(selection.single());
  EXPECT_FALSE(selection.ready());

  selection.click(1.0, 2.0);
  EXPECT_TRUE(selection.has_goal());
  EXPECT_TRUE(selection.single());
  EXPECT_FALSE(selection.start().has_value());
  ASSERT_TRUE(selection.goal().has_value());
  EXPECT_DOUBLE_EQ(selection.goal()->x, 1.0);

  selection.click(4.0, 0.5);
  EXPECT_TRUE(selection.has_goal());
  EXPECT_TRUE(selection.two_point());
  ASSERT_TRUE(selection.ready());
  ASSERT_TRUE(selection.start().has_value());
  EXPECT_DOUBLE_EQ(selection.start()->x, 1.0);
  EXPECT_DOUBLE_EQ(selection.goal()->y, 0.5);
}

// 2026-11-03（用户要求改成多点导航）：第三次点击不再重置成新的单点，
// 而是继续把点追加到同一条路线里 —— 第一个是起点，最后一个是终点，
// 中间的全是途经点。
TEST(PointSelection, ThirdAndFurtherClicksAppendToTheSameRoute)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(1.0, 2.0);
  selection.click(4.0, 0.5);
  selection.click(2.0, -1.0);

  EXPECT_TRUE(selection.has_goal());
  EXPECT_TRUE(selection.multi());
  EXPECT_FALSE(selection.single());
  EXPECT_FALSE(selection.two_point());
  EXPECT_TRUE(selection.ready());
  ASSERT_EQ(selection.size(), 3u);
  // 起点仍是最先点的那个，终点变成最后点的那个
  ASSERT_TRUE(selection.start().has_value());
  EXPECT_DOUBLE_EQ(selection.start()->x, 1.0);
  ASSERT_TRUE(selection.goal().has_value());
  EXPECT_DOUBLE_EQ(selection.goal()->x, 2.0);
  EXPECT_DOUBLE_EQ(selection.goal()->y, -1.0);
  // 顺序必须保留：起点 → 途经点 → 终点
  const auto & points = selection.points();
  ASSERT_EQ(points.size(), 3u);
  EXPECT_DOUBLE_EQ(points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(points[1].x, 4.0);
  EXPECT_DOUBLE_EQ(points[2].x, 2.0);

  selection.click(6.0, 3.0);
  ASSERT_EQ(selection.size(), 4u);
  ASSERT_TRUE(selection.goal().has_value());
  EXPECT_DOUBLE_EQ(selection.goal()->x, 6.0);
  EXPECT_DOUBLE_EQ(selection.start()->x, 1.0);

  selection.clear();
  EXPECT_FALSE(selection.has_goal());
  EXPECT_FALSE(selection.start().has_value());
  EXPECT_FALSE(selection.goal().has_value());
  EXPECT_EQ(selection.size(), 0u);
}
