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
#include <perceptive_mpc/costs/BaseElipseCost.h>
#include <perceptive_mpc/costs/InterpolateBaseArmTrajectory.h>

using namespace perceptive_mpc;

BaseElipseCost::BaseElipseCost(BaseElipseCostConfig config)
    : RelaxedBarrierCost(RelaxedBarrierCost::Config{.mu = config.mu, .delta = config.delta}),
      sigma_(config.sigma),
      kinematics_(config.kinematics) {}

BaseElipseCost *BaseElipseCost::clone() const
{
  return new BaseElipseCost(*this);
}

void BaseElipseCost::intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t &state, const ad_dynamic_vector_t &input,
                                              const ad_dynamic_vector_t &parameters, ad_intermediate_cost_vector_t &costValues) const
{ 

  const Eigen::Quaternion<ad_scalar_t> desiredBOrientation(parameters.head<14>().tail<7>().head<4>());
  const Eigen::Matrix<ad_scalar_t, 3, 1>& desiredBPosition = parameters.head<14>().tail<7>().tail<3>();


  const Eigen::Matrix<ad_scalar_t, 3, 1>& currentBPosition = state.head<7>().tail<3>();  
  const Eigen::Quaternion<ad_scalar_t> currentBOrientationQuat(state.head<7>().head<4>());

  Eigen::Matrix<ad_scalar_t, 3, 1> drpy = ocs2::quaternionDistance(currentBOrientationQuat, desiredBOrientation);
  Eigen::Matrix<ad_scalar_t, 3, 1> dxyz = desiredBPosition - currentBPosition;
  
  Eigen::Matrix<ad_scalar_t, 3, 1> elipse_xyth = parameters.tail<3>();  

  costValues = ad_intermediate_cost_vector_t();
  // reach constraint:

  // costValues(0) = sigma_ * (elipse_xyth[0]*elipse_xyth[0] - dxyz[0]*dxyz[0]);
  // costValues(1) = sigma_ * (elipse_xyth[1]*elipse_xyth[1] - dxyz[1]*dxyz[1]);
  // costValues(2) = sigma_ * (elipse_xyth[2]*elipse_xyth[2] - drpy[2]*drpy[2]);

  // costValues(0) = sigma_ * (((ad_scalar_t) 0.04) - (dxyz[0]*dxyz[0]));
  // costValues(1) = sigma_ * (((ad_scalar_t) 0.04) - (dxyz[1]*dxyz[1]));
  // costValues(2) = sigma_ * (((ad_scalar_t) 0.04) - (drpy[2]*drpy[2]));  

  costValues(0) = sigma_ * ((((ad_scalar_t) 0.025) + elipse_xyth[0]) - CppAD::CondExpGt(dxyz[0], (ad_scalar_t)0, dxyz[0], -dxyz[0]));
  costValues(1) = sigma_ * ((((ad_scalar_t) 0.025) + elipse_xyth[1]) - CppAD::CondExpGt(dxyz[1], (ad_scalar_t)0, dxyz[1], -dxyz[1]));
  costValues(2) = sigma_ * ((((ad_scalar_t) 0.025) + elipse_xyth[2]) - CppAD::CondExpGt(drpy[2], (ad_scalar_t)0, drpy[2], -drpy[2]));

  // costValues(0) = sigma_ * (((ad_scalar_t) 0.1) - CppAD::CondExpGt(dxyz[0], (ad_scalar_t)0, dxyz[0], -dxyz[0]));
  // costValues(1) = sigma_ * (((ad_scalar_t) 0.1) - CppAD::CondExpGt(dxyz[1], (ad_scalar_t)0, dxyz[1], -dxyz[1]));
  // costValues(2) = sigma_ * (((ad_scalar_t) 0.1) - CppAD::CondExpGt(drpy[2], (ad_scalar_t)0, drpy[2], -drpy[2]));
  
}

size_t BaseElipseCost::getNumIntermediateParameters() const
{
  return Definitions::REFERENCE_DIM;
}

size_t BaseElipseCost::getNumTerminalParameters() const
{
  return 0;
}

BaseElipseCost::dynamic_vector_t BaseElipseCost::getIntermediateParameters( BaseElipseCost::scalar_t time) const {
  return interpolateReference(time);
}

BaseElipseCost::dynamic_vector_t BaseElipseCost::interpolateReference(
    BaseElipseCost::scalar_t time) const {
  const auto& desiredTimeTrajectory = costDesiredTrajectoriesPtr_->desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectoriesPtr_->desiredStateTrajectory();
  return interpolateBaseArmElipseTrajectory(desiredTimeTrajectory, desiredStateTrajectory, time);
}

// if x > 0, exp1; else exp2
//  CppAD::CondExpGt(x, azero, <exp1>, <exp2>);

void BaseElipseCost::getIntermediateCostDerivativeTime(scalar_t &dLdt)
{
  dLdt = 0;
}
void BaseElipseCost::getIntermediateCostDerivativeInput(input_vector_t &dLdu)
{
  dLdu = input_vector_t::Zero();
}
void BaseElipseCost::getIntermediateCostSecondDerivativeInput(input_matrix_t &dLduu)
{
  dLduu = input_matrix_t::Zero();
}
void BaseElipseCost::getIntermediateCostDerivativeInputState(input_state_matrix_t &dLdux)
{
  dLdux = input_state_matrix_t::Zero();
}
void BaseElipseCost::getTerminalCost(scalar_t &Phi)
{
  Phi = 0;
}
void BaseElipseCost::getTerminalCostDerivativeTime(scalar_t &dPhidt)
{
  dPhidt = 0;
}
void BaseElipseCost::getTerminalCostDerivativeState(state_vector_t &dPhidx)
{
  dPhidx = state_vector_t::Zero();
}
void BaseElipseCost::getTerminalCostSecondDerivativeState(state_matrix_t &dPhidxx)
{
  dPhidxx = state_matrix_t::Zero();
}
