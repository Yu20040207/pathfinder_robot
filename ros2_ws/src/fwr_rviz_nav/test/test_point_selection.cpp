#include <gtest/gtest.h>
#include "fwr_rviz_nav/point_selection.hpp"

TEST(PointSelection, TwoClicksSelectStartAndGoalThenRestart)
{
  fwr_rviz_nav::PointSelection selection;
  EXPECT_FALSE(selection.ready());
  selection.click(1.0, 2.0);
  ASSERT_TRUE(selection.start().has_value());
  EXPECT_FALSE(selection.goal().has_value());
  selection.click(4.0, 0.5);
  ASSERT_TRUE(selection.ready());
  EXPECT_DOUBLE_EQ(selection.start()->x, 1.0);
  EXPECT_DOUBLE_EQ(selection.goal()->y, 0.5);
  selection.click(2.0, -1.0);
  EXPECT_FALSE(selection.ready());
  EXPECT_DOUBLE_EQ(selection.start()->x, 2.0);
  EXPECT_FALSE(selection.goal().has_value());
  selection.clear();
  EXPECT_FALSE(selection.start().has_value());
}
