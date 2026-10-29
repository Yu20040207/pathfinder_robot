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

TEST(NavigationRoute, SingleClickPlansFromCurrentPoseToGoal)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(2.0, -0.3);

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  const auto preview = fwr_rviz_nav::make_single_preview_goal(selection, stamp);
  ASSERT_TRUE(preview.has_value());
  EXPECT_FALSE(preview->use_start);
  EXPECT_EQ(preview->goal.header.frame_id, "map");
  EXPECT_EQ(preview->goal.header.stamp.sec, 7);
  EXPECT_DOUBLE_EQ(preview->goal.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(preview->goal.pose.position.y, -0.3);
  EXPECT_DOUBLE_EQ(preview->goal.pose.orientation.w, 1.0);

  const auto navigation = fwr_rviz_nav::make_single_navigation_goal(selection, stamp);
  ASSERT_TRUE(navigation.has_value());
  EXPECT_EQ(navigation->pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(navigation->pose.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(navigation->pose.pose.position.y, -0.3);
}

TEST(NavigationRoute, SingleGoalHelpersRejectTwoPointSelection)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(1.0, 2.0);
  selection.click(4.0, 0.5);
  const auto stamp = builtin_interfaces::msg::Time();
  EXPECT_FALSE(fwr_rviz_nav::make_single_preview_goal(selection, stamp).has_value());
  EXPECT_FALSE(fwr_rviz_nav::make_single_navigation_goal(selection, stamp).has_value());
}


TEST(NavigationRoute, RealPreviewStartsAtCurrentRobotThenSelectedStart)
{
  fwr_rviz_nav::PointSelection selection;
  selection.click(0.3, -0.1);
  selection.click(0.6, -0.1);
  builtin_interfaces::msg::Time stamp;
  const auto legs = fwr_rviz_nav::make_real_preview_legs(selection, stamp);
  ASSERT_TRUE(legs.has_value());
  EXPECT_FALSE((*legs)[0].use_start);
  EXPECT_DOUBLE_EQ((*legs)[0].goal.pose.position.x, 0.3);
  EXPECT_TRUE((*legs)[1].use_start);
  EXPECT_DOUBLE_EQ((*legs)[1].start.pose.position.x, 0.3);
  EXPECT_DOUBLE_EQ((*legs)[1].goal.pose.position.x, 0.6);
}

TEST(NavigationRoute, GuardedPreviewMeasuresLongDetourLeg)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped point;
  point.pose.position.x = 0.0;
  path.poses.push_back(point);
  point.pose.position.x = 0.3;
  path.poses.push_back(point);
  point.pose.position.y = 0.4;
  path.poses.push_back(point);
  EXPECT_GT(fwr_rviz_nav::path_length_m(path), 0.5);
}

TEST(NavigationRoute, GuardedPreviewAllowsLongPlannedLegs)
{
  EXPECT_TRUE(fwr_rviz_nav::real_preview_legs_planned(0.4, 0.2));
  EXPECT_TRUE(fwr_rviz_nav::real_preview_legs_planned(4.0, 3.0));
  EXPECT_FALSE(fwr_rviz_nav::real_preview_legs_planned(INFINITY, 3.0));
}
