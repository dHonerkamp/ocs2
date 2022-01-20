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

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include "nav_msgs/OccupancyGrid.h"

namespace ocs2 {
namespace mobile_manipulator {

class MyMap {
public:
    explicit MyMap(std::string topic, scalar_t timeout);
    bool worldToMap(scalar_t wx, scalar_t wy, int &mx, int &my) const;
    int getIndex(int mx, int my) const;
    scalar_t getCost(scalar_t wx, scalar_t wy) const;

    unsigned char* costmap_;
    int size_x_;
    int size_y_;
    scalar_t resolution_;
    scalar_t origin_x_;
    scalar_t origin_y_;
};

class CollisionConstraintSoft final : public StateConstraint {
 public:
  explicit CollisionConstraintSoft(scalar_t radius);
  ~CollisionConstraintSoft() override = default;
  CollisionConstraintSoft* clone() const override { return new CollisionConstraintSoft(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

 private:
  CollisionConstraintSoft(const CollisionConstraintSoft& other) = default;
  vector_t getSdfValueAndGradients(const scalar_t &x, const scalar_t &y) const ;

  scalar_t radius_;
  MyMap sdf_;
  MyMap sdf_dx_;
  MyMap sdf_dy_;
};


}  // namespace mobile_manipulator
}  // namespace ocs2
