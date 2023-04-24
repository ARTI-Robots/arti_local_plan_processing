/*
Created by clemens on 21.04.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <arti_local_plan_processing/acceleration_post_processing.h>

TEST(AccelerationPostProcessingTest, PositiveRamp)
{
  arti_local_plan_processing::AccelerationPostProcessing post_processing;
  post_processing.initialize("post_processing", nullptr, nullptr);

  arti_nav_core_msgs::Trajectory2DWithLimits old_trajectory;
  for (size_t i = 0; i < 10; ++i)
  {
    arti_nav_core_msgs::Movement2DWithLimits movement_1;
    movement_1.pose.point.x.value = static_cast<double>(i) * 0.25;
    movement_1.pose.point.y.value = 0.;
    movement_1.pose.theta.value = 0.;
    movement_1.twist.x.value = 1. * (i > 0 ? 1. : 0.);
    old_trajectory.movements.push_back(movement_1);
  }

  arti_nav_core_msgs::Twist2DWithLimits final_twist;
  arti_nav_core_msgs::Trajectory2DWithLimits new_trajectory;
  arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum result = post_processing.makeTrajectory(
    old_trajectory, final_twist, new_trajectory);

  ASSERT_FLOAT_EQ(new_trajectory.movements[0].twist.x.value, 0.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[1].twist.x.value, 0.70710677);
  ASSERT_FLOAT_EQ(new_trajectory.movements[2].twist.x.value, 1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[3].twist.x.value, 1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[4].twist.x.value, 1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[5].twist.x.value, 1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[6].twist.x.value, 1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[7].twist.x.value, 1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[8].twist.x.value, 0.70710677);
  ASSERT_FLOAT_EQ(new_trajectory.movements[9].twist.x.value, 0.);
  ASSERT_EQ(result,
            arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED);
}

TEST(AccelerationPostProcessingTest, NegativeRamp)
{
  arti_local_plan_processing::AccelerationPostProcessing post_processing;
  post_processing.initialize("post_processing", nullptr, nullptr);

  arti_nav_core_msgs::Trajectory2DWithLimits old_trajectory;
  for (size_t i = 0; i < 10; ++i)
  {
    arti_nav_core_msgs::Movement2DWithLimits movement_1;
    movement_1.pose.point.x.value = static_cast<double>(i) * 0.25;
    movement_1.pose.point.y.value = 0.;
    movement_1.pose.theta.value = 0.;
    movement_1.twist.x.value = -1. * (i > 0 ? 1. : 0.);
    old_trajectory.movements.push_back(movement_1);
  }

  arti_nav_core_msgs::Twist2DWithLimits final_twist;
  arti_nav_core_msgs::Trajectory2DWithLimits new_trajectory;
  arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum result = post_processing.makeTrajectory(
    old_trajectory, final_twist, new_trajectory);

  ASSERT_FLOAT_EQ(new_trajectory.movements[0].twist.x.value, 0.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[1].twist.x.value, -0.70710677);
  ASSERT_FLOAT_EQ(new_trajectory.movements[2].twist.x.value, -1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[3].twist.x.value, -1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[4].twist.x.value, -1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[5].twist.x.value, -1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[6].twist.x.value, -1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[7].twist.x.value, -1.);
  ASSERT_FLOAT_EQ(new_trajectory.movements[8].twist.x.value, -0.70710677);
  ASSERT_FLOAT_EQ(new_trajectory.movements[9].twist.x.value, 0.);
  ASSERT_EQ(result,
            arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED);
}

/**
 * This is currently just here so that some compiled code includes the package's header files, which helps IDEs display
 * them correctly.
 */
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "acceleration_post_processing_test");
  return RUN_ALL_TESTS();
}
