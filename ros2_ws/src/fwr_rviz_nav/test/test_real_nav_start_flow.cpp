#include <gtest/gtest.h>

#include "fwr_rviz_nav/real_nav_start_flow.hpp"

using fwr_rviz_nav::RealNavStartFlow;

TEST(RealNavStartFlow, RequiresGateThenReadyThenArmThenGoal)
{
  RealNavStartFlow flow;
  EXPECT_TRUE(flow.begin());
  EXPECT_FALSE(flow.begin());
  EXPECT_FALSE(flow.ready());
  EXPECT_TRUE(flow.gate_opened(true));
  EXPECT_TRUE(flow.ready());
  EXPECT_TRUE(flow.armed(true));
  EXPECT_TRUE(flow.goal_accepted(true));
  EXPECT_TRUE(flow.active());
  EXPECT_TRUE(flow.stop());
  EXPECT_FALSE(flow.active());
}

TEST(RealNavStartFlow, FailureAndStopAlwaysReturnToIdle)
{
  RealNavStartFlow flow;
  ASSERT_TRUE(flow.begin());
  EXPECT_FALSE(flow.gate_opened(false));
  EXPECT_FALSE(flow.active());
  ASSERT_TRUE(flow.begin());
  ASSERT_TRUE(flow.gate_opened(true));
  EXPECT_TRUE(flow.stop());
  EXPECT_FALSE(flow.ready());
  ASSERT_TRUE(flow.begin());
  ASSERT_TRUE(flow.gate_opened(true));
  ASSERT_TRUE(flow.ready());
  EXPECT_FALSE(flow.armed(false));
  EXPECT_FALSE(flow.active());
}

TEST(RealNavStartFlow, IgnoresOldStoppedSampleUntilTaskActuallyRuns)
{
  RealNavStartFlow flow;
  ASSERT_TRUE(flow.begin());
  ASSERT_TRUE(flow.gate_opened(true));
  ASSERT_TRUE(flow.ready());
  ASSERT_TRUE(flow.armed(true));
  ASSERT_TRUE(flow.goal_accepted(true));
  EXPECT_FALSE(flow.explore_state(true));
  EXPECT_FALSE(flow.explore_state(false));
  EXPECT_TRUE(flow.explore_state(true));
}
