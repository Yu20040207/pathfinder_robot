#include <gtest/gtest.h>

#include "fwr_rviz_nav/navigation_route.hpp"

TEST(NavigationRoute, VisitsSelectedStartBeforeSelectedGoal)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(2.0, -0.3);
  selection.click(6.0, 0.4);

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  const auto legs = fwr_rviz_nav::make_navigation_legs(selection, stamp);

  ASSERT_TRUE(legs.has_value());
  EXPECT_EQ((*legs)[0].pose.header.frame_id, "map");
  EXPECT_EQ((*legs)[0].pose.header.stamp.sec, 42);
  EXPECT_DOUBLE_EQ((*legs)[0].pose.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ((*legs)[0].pose.pose.position.y, -0.3);
  EXPECT_DOUBLE_EQ((*legs)[1].pose.pose.position.x, 6.0);
  EXPECT_DOUBLE_EQ((*legs)[1].pose.pose.position.y, 0.4);
  EXPECT_DOUBLE_EQ((*legs)[0].pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ((*legs)[1].pose.pose.orientation.w, 1.0);
}

TEST(NavigationRoute, DoesNotStartWithOnlyOneSelectedPoint)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(2.0, -0.3);
  EXPECT_FALSE(fwr_rviz_nav::make_navigation_legs(selection, builtin_interfaces::msg::Time()).has_value());
}
