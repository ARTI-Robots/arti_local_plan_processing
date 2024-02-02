/*
Created by clemens on 20.04.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_LOCAL_PLAN_PROCESSING_ACCELERATION_POST_PROCESSING_H
#define ARTI_LOCAL_PLAN_PROCESSING_ACCELERATION_POST_PROCESSING_H

#include <arti_nav_core/base_local_planner_post_processing.h>
#include <arti_nav_core_utils/robot_information.h>
#include <memory>
#include <ros/node_handle.h>
#include <arti_local_plan_processing/AccelerationPostProcessingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <arti_costmap_collision_checker/costmap_collision_check.h>

namespace arti_local_plan_processing
{
class AccelerationPostProcessing : public arti_nav_core::BaseLocalPlannerPostProcessing
{
public:
  AccelerationPostProcessing() = default;

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  BaseLocalPlannerPostProcessingErrorEnum makeTrajectory(
    const arti_nav_core_msgs::Trajectory2DWithLimits& old_trajectory,
    const arti_nav_core_msgs::Twist2DWithLimits& final_twist,
    arti_nav_core_msgs::Trajectory2DWithLimits& new_trajectory) override;
private:
  static const char LOGGER_NAME[];
  
  void reconfigure(const arti_local_plan_processing::AccelerationPostProcessingConfig& new_config);

  void processTrajectory(std::vector<arti_nav_core_msgs::Movement2DWithLimits>& path_element);

  static double getBrakingDistance(double v_prev, double v_next, double acceleration);
  ros::NodeHandle private_nh_;

  std::unique_ptr<dynamic_reconfigure::Server<arti_local_plan_processing::AccelerationPostProcessingConfig>> cfg_server_;
  arti_local_plan_processing::AccelerationPostProcessingConfig cfg_;
  
  ros::Publisher pub_input_path_;
  ros::Publisher pub_output_path_;
};
}

#endif //ARTI_LOCAL_PLAN_PROCESSING_ACCELERATION_POST_PROCESSING_H
