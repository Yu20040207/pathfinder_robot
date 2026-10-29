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

TEST(PointSelection, ThirdClickStartsNewSinglePointSelection)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(1.0, 2.0);
  selection.click(4.0, 0.5);
  selection.click(2.0, -1.0);

  EXPECT_TRUE(selection.single());
  EXPECT_FALSE(selection.ready());
  EXPECT_FALSE(selection.start().has_value());
  ASSERT_TRUE(selection.goal().has_value());
  EXPECT_DOUBLE_EQ(selection.goal()->x, 2.0);

  selection.clear();
  EXPECT_FALSE(selection.has_goal());
  EXPECT_FALSE(selection.start().has_value());
}
