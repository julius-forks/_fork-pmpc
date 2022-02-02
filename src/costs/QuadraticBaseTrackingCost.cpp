/*
 * Copyright (c) 2020 Johannes Pankert <pankertj@ethz.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of this work nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <perceptive_mpc/costs/QuadraticBaseTrackingCost.h>
#include <perceptive_mpc/costs/InterpolateBaseArmTrajectory.h>

using namespace perceptive_mpc;

QuadraticBaseTrackingCost* QuadraticBaseTrackingCost::clone() const {
  return new QuadraticBaseTrackingCost(*this);
}

void QuadraticBaseTrackingCost::intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state,
                                                                const ad_dynamic_vector_t& input, const ad_dynamic_vector_t& parameters,
                                                                ad_intermediate_cost_vector_t& costValues) const {
  // Get desireds
  const Eigen::Quaternion<ad_scalar_t> desiredBOrientation(parameters.head<14>().tail<7>().head<4>());
  const Eigen::Matrix<ad_scalar_t, 3, 1>& desiredBPosition = parameters.head<14>().tail<7>().tail<3>();


  const Eigen::Matrix<ad_scalar_t, 3, 1>& currentBPosition = state.head<7>().tail<3>();  
  const Eigen::Quaternion<ad_scalar_t> currentBOrientationQuat(state.head<7>().head<4>());

  Eigen::Matrix<ad_scalar_t, 3, 1> orientationBError = ocs2::quaternionDistance(currentBOrientationQuat, desiredBOrientation);
  Eigen::Matrix<ad_scalar_t, 3, 1> positionBError = desiredBPosition - currentBPosition;

  //weights
  Eigen::Matrix<ad_scalar_t, 3, 1> error;
  error << positionBError[0], positionBError[1],orientationBError[2];// only adding base x,y, th


  costValues = ad_intermediate_cost_vector_t();
  costValues << base_R_.array().sqrt().matrix().cast<ad_scalar_t>() * input, base_Q_.array().sqrt().matrix().cast<ad_scalar_t>() * error;
}

void QuadraticBaseTrackingCost::terminalCostFunction(ad_interface_t::ad_scalar_t time,
                                                            const ad_interface_t::ad_dynamic_vector_t& state,
                                                            const ad_interface_t::ad_dynamic_vector_t& parameters,
                                                            ad_terminal_cost_vector_t& costValues) const {

  const Eigen::Quaternion<ad_scalar_t> desiredBOrientation(parameters.head<14>().tail<7>().head<4>());
  const Eigen::Matrix<ad_scalar_t, 3, 1>& desiredBPosition = parameters.head<14>().tail<7>().tail<3>();

  const Eigen::Matrix<ad_scalar_t, 3, 1>& currentBPosition = state.head<7>().tail<3>();  
  const Eigen::Quaternion<ad_scalar_t> currentBOrientationQuat(state.head<7>().head<4>());

  Eigen::Matrix<ad_scalar_t, 3, 1> orientationBError = ocs2::quaternionDistance(currentBOrientationQuat, desiredBOrientation);
  Eigen::Matrix<ad_scalar_t, 3, 1> positionBError = desiredBPosition - currentBPosition;

  //weights
  Eigen::Matrix<ad_scalar_t, 3, 1> error;
  error << positionBError[0], positionBError[1],orientationBError[2];// only adding base x,y, th

  costValues = ad_terminal_cost_vector_t();
  costValues = base_QFinal_.array().sqrt().matrix().cast<ad_scalar_t>() * error;  
}

QuadraticBaseTrackingCost::dynamic_vector_t QuadraticBaseTrackingCost::getIntermediateParameters(
    QuadraticBaseTrackingCost::scalar_t time) const {
  return interpolateReference(time);
}
size_t QuadraticBaseTrackingCost::getNumIntermediateParameters() const {
  return Definitions::REFERENCE_DIM;
}

QuadraticBaseTrackingCost::dynamic_vector_t QuadraticBaseTrackingCost::getTerminalParameters(scalar_t time) const {
  return interpolateReference(time);
}
size_t QuadraticBaseTrackingCost::getNumTerminalParameters() const {
  return Definitions::REFERENCE_DIM;
}

QuadraticBaseTrackingCost::dynamic_vector_t QuadraticBaseTrackingCost::interpolateReference(
    QuadraticBaseTrackingCost::scalar_t time) const {
  const auto& desiredTimeTrajectory = costDesiredTrajectoriesPtr_->desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectoriesPtr_->desiredStateTrajectory();
  return interpolateBaseArmTrajectory(desiredTimeTrajectory, desiredStateTrajectory, time);
}

Eigen::Quaternion<QuadraticBaseTrackingCost::ad_scalar_t> QuadraticBaseTrackingCost::matrixToQuaternion(
    const Eigen::Matrix<ad_scalar_t, 3, 3>& R) const {
  ad_scalar_t t1, t2, t;
  ad_scalar_t x1, x2, x;
  ad_scalar_t y1, y2, y;
  ad_scalar_t z1, z2, z;
  ad_scalar_t w1, w2, w;

  t1 = CppAD::CondExpGt(R(0, 0), R(1, 1), 1 + R(0, 0) - R(1, 1) - R(2, 2), 1 - R(0, 0) + R(1, 1) - R(2, 2));
  t2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), 1 - R(0, 0) - R(1, 1) + R(2, 2), 1 + R(0, 0) + R(1, 1) + R(2, 2));
  t = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), t1, t2);

  x1 = CppAD::CondExpGt(R(0, 0), R(1, 1), t, R(1, 0) + R(0, 1));
  x2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(0, 2) + R(2, 0), R(2, 1) - R(1, 2));
  x = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), x1, x2);

  y1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(1, 0) + R(0, 1), t);
  y2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(2, 1) + R(1, 2), R(0, 2) - R(2, 0));
  y = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), y1, y2);

  z1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(0, 2) + R(2, 0), R(2, 1) + R(1, 2));
  z2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), t, R(1, 0) - R(0, 1));
  z = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), z1, z2);

  w1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(2, 1) - R(1, 2), R(0, 2) - R(2, 0));
  w2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(1, 0) - R(0, 1), t);
  w = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), w1, w2);

  Eigen::Matrix<ad_scalar_t, 4, 1> q({x, y, z, w});
  q *= 0.5 / sqrt(t);

  Eigen::Quaternion<ad_scalar_t> quaternion;
  quaternion.x() = q(0);
  quaternion.y() = q(1);
  quaternion.z() = q(2);
  quaternion.w() = q(3);

  return quaternion;
}
