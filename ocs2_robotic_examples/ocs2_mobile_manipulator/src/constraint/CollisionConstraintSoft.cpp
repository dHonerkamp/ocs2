/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_mobile_manipulator/constraint/CollisionConstraintSoft.h>
#include <iostream>
#include <ros/topic.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

namespace ocs2 {
namespace mobile_manipulator {

MyMap::MyMap(const std::string topic, scalar_t timeout){
  std::cout << "Waiting for " << topic << std::endl;
  const nav_msgs::OccupancyGrid::ConstPtr& map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(topic);

  size_x_ = map->info.width;
  size_y_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  costmap_ = new unsigned char[size_x_ * size_y_];

  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    costmap_[it] = map->data[it];
  }
  std::cout << topic << " received" << std::endl;
}


bool MyMap::worldToMap(scalar_t wx, scalar_t wy, int &mx, int &my) const {
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

int MyMap::getIndex(int mx, int my) const {
  return my * size_x_ + mx;
}

scalar_t MyMap::getCost(scalar_t wx, scalar_t wy) const {
  int mx, my;
  worldToMap(wx, wy, mx, my);
  return static_cast<scalar_t>(costmap_[getIndex(mx, my)]);
}

CollisionConstraintSoft::CollisionConstraintSoft(scalar_t radius) : StateConstraint(ConstraintOrder::Linear),
                                                                    radius_{radius},
                                                                    sdf_{"/sdf", 10.0},
                                                                    sdf_dx_{"/sdf_dx", 10.0},
                                                                    sdf_dy_{"/sdf_dy", 10.0}
                                                                    {
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CollisionConstraintSoft::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  scalar_t current_x = state(0);
  scalar_t current_y = state(1);

  vector_t value_and_gradient = getSdfValueAndGradients(current_x, current_y);
  scalar_t signed_distance = value_and_gradient(0);

  scalar_t z = signed_distance - radius_;

  vector_t value(1);
  value(0) = z;
  return value;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation CollisionConstraintSoft::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                           const PreComputation& preComputation) const {
  VectorFunctionLinearApproximation limits(getNumConstraints(time), state.rows(), 0);

  scalar_t current_x = state(0);
  scalar_t current_y = state(1);
  vector_t value_and_gradient = getSdfValueAndGradients(current_x, current_y);

  limits.f(0) = value_and_gradient(0) - radius_;

  limits.dfdx.setZero();
  limits.dfdx(0, 0) = value_and_gradient(1);
  limits.dfdx(0, 1) = value_and_gradient(2);

  return limits;
}

vector_t CollisionConstraintSoft::getSdfValueAndGradients(const scalar_t &x, const scalar_t &y) const {
  vector_t values(3);
  // NOTE: rescale matching the scaling map.py
  // TODO: rm if we use another way to get the sdf
  values(0) = 0.01 * sdf_.getCost(x, y);
  values(1) = - (0.01 * sdf_dx_.getCost(x, y) - 0.5) * 0.05;
  values(2) = - (0.01 * sdf_dy_.getCost(x, y) - 0.5) * 0.05;

  // TODO: not sure actually needed now
  scalar_t x_rounded = std::round((x - sdf_.origin_x_) / sdf_.resolution_) * sdf_.resolution_ + sdf_.origin_x_;
  scalar_t y_rounded = std::round((y - sdf_.origin_y_) / sdf_.resolution_) * sdf_.resolution_ + sdf_.origin_y_;
  scalar_t sdf_interp = values(0) + (x - x_rounded) * values(1) + (y - y_rounded) * values(2);
  values(0) = sdf_interp;

  return values;
}


}  // namespace mobile_manipulator
}  // namespace ocs2
