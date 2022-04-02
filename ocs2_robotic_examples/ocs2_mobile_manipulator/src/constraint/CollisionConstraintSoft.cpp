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

MyMap::MyMap(const std::string topic, scalar_t timeout, ros::NodeHandle& nodeHandle){
  std::cout << "Waiting for " << topic << std::endl;
  const ocs2_msgs::mysdfgrid::ConstPtr& map = ros::topic::waitForMessage<ocs2_msgs::mysdfgrid>(topic);

  size_x_ = map->info.width;
  size_y_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  costmap_ = new float[size_x_ * size_y_];

  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    costmap_[it] = map->data[it];
  }
  std::cout << topic << " received" << std::endl;

  sub_ = nodeHandle.subscribe(topic, 1, &MyMap::updateCallback, this);
}

void MyMap::updateCallback(const ocs2_msgs::mysdfgrid::ConstPtr& map){
  size_x_ = map->info.width;
  size_y_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  costmap_ = new float[size_x_ * size_y_];

  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    costmap_[it] = map->data[it];
  }
  std::cout << "SDF map received" << std::endl;
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
  bool success = worldToMap(wx, wy, mx, my);
  if (!success){
    std::cout << "Location outside of map, returning 0: wx: " << wx << ", wy: " << wy << ", origin_x_: " << origin_x_ << ", origin_y_: " << origin_y_ << std::endl;
    return 0.0;
  }
  return static_cast<scalar_t>(costmap_[getIndex(mx, my)]);
}

CollisionConstraintSoft::CollisionConstraintSoft(scalar_t radius, bool square_base, scalar_t corner_radius, scalar_t diagonal_radius, ros::NodeHandle& nodeHandle) : StateConstraint(ConstraintOrder::Linear),
                                                                    radius_{radius},
                                                                    square_base_{square_base},
                                                                    corner_radius_{corner_radius},
                                                                    diagonal_radius_{diagonal_radius},
                                                                    sdf_{"/sdf", 10.0, nodeHandle},
                                                                    sdf_dx_{"/sdf_dx", 10.0, nodeHandle},
                                                                    sdf_dy_{"/sdf_dy", 10.0, nodeHandle}
                                                                    {

//    std::vector<double> values = {-5., -4., -3., -2., -1., 0.0, 1., 2., 3., 4., 5.};
//    for (auto x : values){
//      for (auto y : values){
//        getSdfValueAndGradients(x, y);
//      }
//    }

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

  vector_t value(getNumConstraints(time));
  value(0) = z;

  if (square_base_){
    scalar_t current_theta = state(2);
    std::vector<std::vector<scalar_t>> corners = getSquareBaseCorners(current_x, current_y, current_theta);
    value(1) = getSdfValueAndGradients(corners[0][0], corners[0][1])(0) - corner_radius_;
    value(2) = getSdfValueAndGradients(corners[1][0], corners[1][1])(0) - corner_radius_;
    value(3) = getSdfValueAndGradients(corners[2][0], corners[2][1])(0) - corner_radius_;
    value(4) = getSdfValueAndGradients(corners[3][0], corners[3][1])(0) - corner_radius_;
  }

//  std::cout << "value: " << value << std::endl;

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
  // std::cout << "current_x: " << current_x << ", current_y: " << current_y << std::endl;

  limits.f(0) = value_and_gradient(0) - radius_;

  limits.dfdx.setZero();
  limits.dfdx(0, 0) = value_and_gradient(1);
  limits.dfdx(0, 1) = value_and_gradient(2);

  if (square_base_){
    scalar_t current_theta = state(2);
    std::vector<std::vector<scalar_t>> corners = getSquareBaseCorners(current_x, current_y, current_theta);
    vector_t value_and_gradient1 = getSdfValueAndGradients(corners[0][0], corners[0][1]);
    vector_t value_and_gradient2 = getSdfValueAndGradients(corners[1][0], corners[1][1]);
    vector_t value_and_gradient3 = getSdfValueAndGradients(corners[2][0], corners[2][1]);
    vector_t value_and_gradient4 = getSdfValueAndGradients(corners[3][0], corners[3][1]);

    limits.f(1) = value_and_gradient1(0) - corner_radius_;
    limits.f(2) = value_and_gradient2(0) - corner_radius_;
    limits.f(3) = value_and_gradient3(0) - corner_radius_;
    limits.f(4) = value_and_gradient4(0) - corner_radius_;

    limits.dfdx(1, 0) = value_and_gradient1(1) ;
    limits.dfdx(1, 1) = value_and_gradient1(2) ;
    scalar_t r = diagonal_radius_;
    limits.dfdx(1, 2) = (- r * std::sin(current_theta + 0.25 * M_PI) * value_and_gradient1(1) + r * std::cos(current_theta + 0.25 * M_PI) * value_and_gradient1(2));
    limits.dfdx(2, 0) = value_and_gradient2(1);
    limits.dfdx(2, 1) = value_and_gradient2(2);
    limits.dfdx(2, 2) = (- r * std::sin(current_theta + 0.75 * M_PI) * value_and_gradient2(1) + r * std::cos(current_theta + 0.75 * M_PI) * value_and_gradient2(2));
    limits.dfdx(3, 0) = value_and_gradient3(1);
    limits.dfdx(3, 1) = value_and_gradient3(2);
    limits.dfdx(3, 2) = (- r * std::sin(current_theta + 1.25 * M_PI) * value_and_gradient3(1) + r * std::cos(current_theta + 1.25 * M_PI) * value_and_gradient3(2));
    limits.dfdx(4, 0) = value_and_gradient4(1);
    limits.dfdx(4, 1) = value_and_gradient4(2);
    limits.dfdx(4, 2) = (- r * std::sin(current_theta + 1.75 * M_PI) * value_and_gradient4(1) + r * std::cos(current_theta + 1.75 * M_PI) * value_and_gradient4(2));
  }

//  std::cout << "limits: " << limits << std::endl;

  return limits;
}

vector_t CollisionConstraintSoft::getSdfValueAndGradients(const scalar_t &x, const scalar_t &y) const {
  vector_t values(3);
  // NOTE: rescale matching the scaling map.py
  values(0) = sdf_.getCost(x, y);
  values(1) = sdf_dx_.getCost(x, y);
  values(2) = sdf_dy_.getCost(x, y);

  // if (values(0) < 1e-3){
  //   std::cout << "x: " << x << ", y: " << y << ", cost: " << values(0) << ", dx: " << values(1) << ", dy: " << values(2) << std::endl;
  // }

  // TODO: not sure actually needed now
  scalar_t x_rounded = std::round((x - sdf_.origin_x_) / sdf_.resolution_) * sdf_.resolution_ + sdf_.origin_x_;
  scalar_t y_rounded = std::round((y - sdf_.origin_y_) / sdf_.resolution_) * sdf_.resolution_ + sdf_.origin_y_;
  scalar_t sdf_interp = values(0) + (x - x_rounded) * values(1) + (y - y_rounded) * values(2);
  values(0) = sdf_interp;

  return values;
}

std::vector<std::vector<scalar_t>> CollisionConstraintSoft::getSquareBaseCorners(scalar_t &current_x, scalar_t &current_y, scalar_t &current_theta) const{
  std::vector<std::vector<scalar_t>> corners(4);
  scalar_t r = diagonal_radius_;
  corners[0] = std::vector<scalar_t>{current_x + r * std::cos(current_theta + 0.25 * M_PI), current_y + r * std::sin(current_theta + 0.25 * M_PI)};
  corners[1] = std::vector<scalar_t>{current_x + r * std::cos(current_theta + 0.75 * M_PI), current_y + r * std::sin(current_theta + 0.75 * M_PI)};
  corners[2] = std::vector<scalar_t>{current_x + r * std::cos(current_theta + 1.25 * M_PI), current_y + r * std::sin(current_theta + 1.25 * M_PI)};
  corners[3] = std::vector<scalar_t>{current_x + r * std::cos(current_theta + 1.75 * M_PI), current_y + r * std::sin(current_theta + 1.75 * M_PI)};

//  std::cout<< "current_x: " << current_x << ", current_y: " << current_y << ", current_theta: " << current_theta << ", c0: (" << corners[0][0] << "," << corners[0][1] << ")" << ", c1: (" << corners[1][0] << "," << corners[1][1] << ")" << ", c2: (" << corners[2][0] << "," << corners[2][1] << ")" << ", c0: (" << corners[3][0] << "," << corners[3][1] << ")" << std::endl;

  return corners;
}


}  // namespace mobile_manipulator
}  // namespace ocs2
