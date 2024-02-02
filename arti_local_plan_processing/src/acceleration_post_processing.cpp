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
const char AccelerationPostProcessing::LOGGER_NAME[] = "acceleration_post_processing";

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
  new_trajectory.header = old_trajectory.header;
  new_trajectory.movements.clear();

  if (old_trajectory.movements.empty())
  {
    new_trajectory = old_trajectory;
    ROS_ERROR("Post Processing called with empty trajectory");
    return arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED;
  }

  double distance = 0.0;
  auto start_path = old_trajectory.movements.begin();
  for (size_t i = 1; i < old_trajectory.movements.size() - 1; ++i)
  {
    float current_speed = old_trajectory.movements[i-1].twist.x.value;
    float next_speed = old_trajectory.movements[i].twist.x.value;
    if (std::signbit(current_speed) == std::signbit(next_speed))
    {
      distance += std::hypot(
        old_trajectory.movements[i].pose.point.x.value - old_trajectory.movements[i-1].pose.point.x.value,
        old_trajectory.movements[i].pose.point.y.value - old_trajectory.movements[i-1].pose.point.y.value);

    }
    else
    {
      ROS_DEBUG("split trajectory at [%zu]", i);
      //split here
      std::vector<arti_nav_core_msgs::Movement2DWithLimits> path_element(start_path, (old_trajectory.movements.begin()+i));
      if(!new_trajectory.movements.empty())
      {
        path_element.begin()->twist.x.value = new_trajectory.movements.back().twist.x.value;
        std::copysignf(path_element.begin()->twist.x.value, (path_element.begin()+1)->twist.x.value );
      }
      path_element.back().twist.x.value = (( path_element.back().twist.x.value >= 0. ? 1. : -1.0) * 0.0); // set last velocity to (almost) zero
      for(size_t j = 0; j < path_element.size(); ++j)
      {
        ROS_DEBUG("path_element before processing [%f] ", path_element[j].twist.x.value);
      }
      processTrajectory(path_element);
      if(!new_trajectory.movements.empty())
      {
        new_trajectory.movements.pop_back(); // we don't want to append the switch 2 times
      }
      new_trajectory.movements.insert(new_trajectory.movements.end(), path_element.begin(), path_element.end());
      start_path = old_trajectory.movements.begin()+i-1;

    }

  }

  ROS_DEBUG("Acceleration post processing, last path element");

  std::vector<arti_nav_core_msgs::Movement2DWithLimits> path_element(start_path, old_trajectory.movements.end());
  //path_element.back().twist.x.value = ( path_element.back().twist.x.value >= 0. ? 1. : -1.0) * 0.0; // set last velocity to (almost) zero
  if(!new_trajectory.movements.empty())
  {
    path_element.begin()->twist.x.value = new_trajectory.movements.back().twist.x.value;
    std::copysignf(path_element.begin()->twist.x.value, (path_element.begin()+1)->twist.x.value );
  }
  for(size_t i = 0; i < path_element.size(); ++i)
  {
    ROS_DEBUG("last pathelement before processing [%f] ", path_element[i].twist.x.value);
  }
  processTrajectory(path_element);
  if(!new_trajectory.movements.empty())
  {
    new_trajectory.movements.pop_back(); // we don't want to append the switch 2 times
  }
  for(size_t i = 0; i < path_element.size(); ++i)
  {
    ROS_INFO("last path elements after processing [%f] ", path_element[i].twist.x.value);
  }
  new_trajectory.movements.insert(new_trajectory.movements.end(), path_element.begin(), path_element.end());
  new_trajectory.movements.back().twist = final_twist;
  
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

void AccelerationPostProcessing::processTrajectory(std::vector<arti_nav_core_msgs::Movement2DWithLimits>& path_element)
{
  double distance = 0.0;
  for (size_t i = 1; i < path_element.size() - 1; ++i)
  {
    if (!std::isfinite(path_element[i].twist.x.value))
    {
      continue;
    }
    const bool positive_speed =
      (path_element[0].twist.x.value >= 0.) && (path_element[i].twist.x.value >= 0.);
    const bool accelerate =
      ((path_element[i].twist.x.value - path_element[0].twist.x.value ) > 0.);
    const double acceleration_used = positive_speed ? cfg_.acceleration : cfg_.deceleration; // why use deceleration value when driving in reverse?

      // incrementally update distance
      distance += std::hypot(
        path_element[i].pose.point.x.value - path_element[i-1].pose.point.x.value,
        path_element[i].pose.point.y.value - path_element[i-1].pose.point.y.value);

      //breaking and acceleration distance is calculated the same way there would only be a difference in the sign of the distance
      double acceleration_distance = getBrakingDistance(path_element[i].twist.x.value,
                                                        path_element[0].twist.x.value, acceleration_used);
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME,
                             "distance: " << distance << " acceleration_distance: " << acceleration_distance);

      // only update the acceleration ramp as long the acc ramp distance is smaller than the traveld path
      if (distance < acceleration_distance)
      {
        float update = (path_element[i].twist.x.value >= 0 ? 1. : -1.) *
        std::sqrt(std::pow(path_element[0].twist.x.value, 2.) +
                  distance * 2. * acceleration_used);
        ROS_DEBUG("index [%zu] velocity curr [%f] updated [%f]", i, path_element[i].twist.x.value, update);
        path_element[i].twist.x.value =
          (path_element[i].twist.x.value >= 0 ? 1. : -1.) *
          std::sqrt(std::pow(path_element[0].twist.x.value, 2.) +
                    distance * 2. * acceleration_used);

      }
      else
      {
        break;
      }
    }

  distance = 0.0;
  for (size_t i = path_element.size() - 2; i > 0; --i)
  {
    if (!std::isfinite(path_element[i].twist.x.value))
    {
      continue;
    }
    const size_t last_index = path_element.size()-1;
    const bool positive_speed =
      (path_element[i - 1].twist.x.value >= 0.) && (path_element[i].twist.x.value >= 0.);

    const auto speed_difference = (path_element[i].twist.x.value
                                   - path_element[last_index].twist.x.value);
    const bool slowing_down = positive_speed ? (speed_difference < 0.) : (speed_difference > 0.);

    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "end_speed: " << path_element[last_index].twist.x.value << " curr_vel: "
                                                     << path_element[i].twist.x.value
                                                     << " speed_difference: "
                                                     << speed_difference << " slowing_down: "
                                                     << static_cast<int>(slowing_down));

    //if (!slowing_down)
    //{
    //  continue;
    //}

    const double deceleration_used = positive_speed ? cfg_.deceleration : cfg_.acceleration;


    distance += std::hypot(
      path_element[i].pose.point.x.value - path_element[i+1].pose.point.x.value,
      path_element[i].pose.point.y.value - path_element[i+1].pose.point.y.value);

    double breaking_distance = getBrakingDistance(path_element[i].twist.x.value,
                                                  path_element[last_index].twist.x.value, deceleration_used);
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "Breaking, distance: " << distance << " breaking_distance: " << breaking_distance);

    if (distance < breaking_distance)
    {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "speed: " << path_element[i].twist.x.value);
      //should slow down in one step more than the deceleration
      //path_element[k].twist.x.value = (positive_speed ? 1. : -1.) * std::sqrt(
      //  std::pow(path_element[i].twist.x.value, 2.) + distance * 2. * deceleration_used);

      float update = (path_element[i].twist.x.value >= 0 ? 1. : -1.) *
                     std::sqrt(std::pow(path_element[0].twist.x.value, 2.) +
                               distance * 2. * deceleration_used);
      ROS_DEBUG("index [%zu] velocity curr [%f] updated [%f]",i, path_element[i].twist.x.value, update);
      path_element[i].twist.x.value =
        (path_element[i].twist.x.value >= 0. ? 1. : -1.) * std::sqrt(
          std::pow(path_element[last_index].twist.x.value, 2.) + distance * 2. * deceleration_used);

      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "speed: " << path_element[i].twist.x.value);
    }
    else
    {
      break;
    }
  }
}

double AccelerationPostProcessing::getBrakingDistance(double v_prev, double v_next, double acceleration)
{
  if (acceleration < 0.001)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "acceleration limit 0.001, given value: " << acceleration << " overridden");
    acceleration = 0.001;
  }
  double breaking_distance = std::abs(std::pow(v_prev, 2) - std::pow(v_next, 2)) / (2 * acceleration);
  return breaking_distance;
}
}

PLUGINLIB_EXPORT_CLASS(arti_local_plan_processing::AccelerationPostProcessing,
  arti_nav_core::BaseLocalPlannerPostProcessing
)
