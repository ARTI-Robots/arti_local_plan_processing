/*
Created by clemens on 20.04.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_local_plan_processing/acceleration_post_processing.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <arti_nav_core_utils/conversions.h>

namespace arti_local_plan_processing
{
void AccelerationPostProcessing::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros)
{
  private_nh_ = ros::NodeHandle("~/" + name);

  pub_input_path_ = private_nh_.advertise<nav_msgs::Path>("input_path", 1, true);
  pub_output_path_ = private_nh_.advertise<nav_msgs::Path>("output_path", 1, true);

  cfg_server_.reset(
    new dynamic_reconfigure::Server<arti_local_plan_processing::AccelerationPostProcessingConfig>(private_nh_));
  cfg_server_->setCallback(std::bind(&AccelerationPostProcessing::reconfigure, this, std::placeholders::_1));
}

arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum AccelerationPostProcessing::makeTrajectory(
  const arti_nav_core_msgs::Trajectory2DWithLimits& old_trajectory,
  const arti_nav_core_msgs::Twist2DWithLimits& final_twist,
  arti_nav_core_msgs::Trajectory2DWithLimits& new_trajectory)
{
  new_trajectory = old_trajectory;

  if (new_trajectory.movements.empty())
  {
    return arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED;
  }

  new_trajectory.movements.back().twist = final_twist;

  for (size_t i = 0; i < new_trajectory.movements.size() - 1; ++i)
  {
    if (!std::isfinite(new_trajectory.movements[i].twist.x.value))
    {
      continue;
    }

    const bool positive_speed =
      (new_trajectory.movements[i].twist.x.value >= 0.) && (new_trajectory.movements[i + 1].twist.x.value >= 0.);

    const auto speed_difference = (new_trajectory.movements[i + 1].twist.x.value
                                   - new_trajectory.movements[i].twist.x.value);
    const bool speed_up = positive_speed ? (speed_difference > 0.) : (speed_difference < 0.);

    ROS_DEBUG_STREAM("prev_vel: " << new_trajectory.movements[i].twist.x.value << " next_vel: "
                                  << new_trajectory.movements[i + 1].twist.x.value << " speed_difference: "
                                  << speed_difference << " speed_up: " << static_cast<int>(speed_up));

    if (!speed_up)
    {
      continue;
    }

    const double acceleration_used = positive_speed ? cfg_.acceleration : cfg_.deceleration;

    for (size_t k = i + 1; k < new_trajectory.movements.size(); ++k)
    {
      const auto distance = std::hypot(
        new_trajectory.movements[k].pose.point.x.value - new_trajectory.movements[i].pose.point.x.value,
        new_trajectory.movements[k].pose.point.y.value - new_trajectory.movements[i].pose.point.y.value);

      double acceleration_distance = getBrakingDistance(new_trajectory.movements[i].twist.x.value,
                                                        new_trajectory.movements[k].twist.x.value, acceleration_used);
      ROS_DEBUG_STREAM("distance: " << distance << " acceleration_distance: " << acceleration_distance);

      if (distance < acceleration_distance)
      {
        //should speed up one step more then acceleration
        new_trajectory.movements[k].twist.x.value = (positive_speed ? 1. : -1.) * std::sqrt(
          std::pow(new_trajectory.movements[i].twist.x.value, 2.) + distance * 2. * acceleration_used);
      }
      else
      {
        break;
      }
    }
  }

  for (size_t i = new_trajectory.movements.size() - 1; i > 0; --i)
  {
    if (!std::isfinite(new_trajectory.movements[i].twist.x.value))
    {
      continue;
    }

    const bool positive_speed =
      (new_trajectory.movements[i - 1].twist.x.value >= 0.) && (new_trajectory.movements[i].twist.x.value >= 0.);

    const auto speed_difference = (new_trajectory.movements[i].twist.x.value
                                   - new_trajectory.movements[i - 1].twist.x.value);
    const bool slowing_down = positive_speed ? (speed_difference < 0.) : (speed_difference > 0.);

    ROS_DEBUG_STREAM("prev_vel: " << new_trajectory.movements[i - 1].twist.x.value << " next_vel: "
                                  << new_trajectory.movements[i].twist.x.value << " speed_difference: "
                                  << speed_difference << " slowing_down: " << static_cast<int>(slowing_down));

    if (!slowing_down)
    {
      continue;
    }

    const double deceleration_used = positive_speed ? cfg_.deceleration : cfg_.acceleration;

    for (size_t k = i - 1; k > 0; --k)
    {
      const auto distance = std::hypot(
        new_trajectory.movements[k].pose.point.x.value - new_trajectory.movements[i].pose.point.x.value,
        new_trajectory.movements[k].pose.point.y.value - new_trajectory.movements[i].pose.point.y.value);

      double breaking_distance = getBrakingDistance(new_trajectory.movements[k].twist.x.value,
                                                    new_trajectory.movements[i].twist.x.value, deceleration_used);
      ROS_DEBUG_STREAM("distance: " << distance << " breaking_distance: " << breaking_distance);

      if (distance < breaking_distance)
      {
        ROS_DEBUG_STREAM("speed: " << new_trajectory.movements[k].twist.x.value);
        //should slow down in one step more then the deceleration
        new_trajectory.movements[k].twist.x.value = (positive_speed ? 1. : -1.) * std::sqrt(
          std::pow(new_trajectory.movements[i].twist.x.value, 2.) + distance * 2. * deceleration_used);

        ROS_DEBUG_STREAM("speed: " << new_trajectory.movements[k].twist.x.value);
      }
      else
      {
        break;
      }
    }
  }

  // publish input path (convert to path without limits)
  nav_msgs::Path input_path = arti_nav_core_utils::convertToPath(old_trajectory,
                                                                 arti_nav_core_utils::non_finite_values::THROW);
  pub_input_path_.publish(input_path);

  nav_msgs::Path output_path = arti_nav_core_utils::convertToPath(new_trajectory,
                                                                  arti_nav_core_utils::non_finite_values::THROW);
  pub_output_path_.publish(output_path);

  return arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED;
}

void AccelerationPostProcessing::reconfigure(
  const arti_local_plan_processing::AccelerationPostProcessingConfig& new_config)
{
  cfg_ = new_config;
}

double AccelerationPostProcessing::getBrakingDistance(double v_prev, double v_next, double acceleration)
{
  if (acceleration < 0.001)
  {
    ROS_WARN_STREAM("acceleration limit 0.001, given value: " << acceleration << " overridden");
    acceleration = 0.001;
  }
  double breaking_distance = std::abs(std::pow(v_prev, 2) - std::pow(v_next, 2)) / (2 * acceleration);
  return breaking_distance;
}
}

PLUGINLIB_EXPORT_CLASS(arti_local_plan_processing::AccelerationPostProcessing,
                       arti_nav_core::BaseLocalPlannerPostProcessing)
