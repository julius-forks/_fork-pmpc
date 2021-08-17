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

#include <iit/rbd/rbd.h>
#include <iit/rbd/traits/TraitSelector.h>
#include "generated/transforms.h"

#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

using namespace perceptive_mpc;

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 4> asArmKinematics<SCALAR_T>::computeArmMountToToolMountTransform(
    const Eigen::Matrix<SCALAR_T, 6, 1>& armState) const {
  typedef typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait trait_t;
  typename iit::asArm::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarm_mount_X_fr_xarmlink3 armmountTolink3;
  typename iit::asArm::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarmlink3_X_fr_xarmlink6 link3Tolink6;
  const Eigen::Matrix<SCALAR_T, 4, 4> armmountTolink6 = armmountTolink3.update(armState) * link3Tolink6.update(armState);
  return armmountTolink6;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, -1> asArmKinematics<SCALAR_T>::computeArmState2MultiplePointsOnRobot(
    const Eigen::Matrix<SCALAR_T, 6, 1>& state, const std::vector<std::vector<double>>& points,
    const Eigen::Matrix4d& transformBase_X_ArmBase, const Eigen::Matrix4d& transformToolMount_X_Endeffector,
    const Eigen::Matrix<SCALAR_T, 4, 4>& transformWorld_X_Base) const {
  assert(points.size() == 8);

  int dim = 0;
  for (int i = 0; i < points.size(); i++) {
    for (int j = 0; j < points[i].size(); j++) {
      dim++;
    }
  }
  if (dim == 0) {
    return Eigen::Matrix<SCALAR_T, 3, -1>(3, 0);
  }
  Eigen::Matrix<SCALAR_T, 3, -1> result(3, dim);
  int resultIndex = 0;
  int linkIndex;

  Eigen::Matrix<SCALAR_T, 4, 4> transformWorld_X_Endeffector = transformWorld_X_Base;

  typedef typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait trait_t;
  typename iit::ur10::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarm_mount_X_fr_xarmlink1 fr_xarm_mount_X_fr_xarmlink1;
  typename iit::ur10::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarmlink1_X_fr_xarmlink2 fr_xarmlink1_X_fr_xarmlink2;
  typename iit::ur10::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarmlink2_X_fr_xarmlink3 fr_xarmlink2_X_fr_xarmlink3;
  typename iit::ur10::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarmlink3_X_fr_xarmlink4 fr_xarmlink3_X_fr_xarmlink4;
  typename iit::ur10::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarmlink4_X_fr_xarmlink5 fr_xarmlink4_X_fr_xarmlink5;
  typename iit::ur10::tpl::HomogeneousTransforms<trait_t>::Type_fr_xarmlink5_X_fr_xarmlink6 fr_xarmlink5_X_fr_xarmlink6;

  Eigen::Matrix<SCALAR_T, 4, 4> nextStep = transformBase_X_ArmBase.cast<SCALAR_T>();
  
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = fr_xarm_mount_X_fr_xarmlink1.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = fr_xarmlink1_X_fr_xarmlink2.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = fr_xarmlink2_X_fr_xarmlink3.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = fr_xarmlink3_X_fr_xarmlink4.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = fr_xarmlink4_X_fr_xarmlink5.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = fr_xarmlink5_X_fr_xarmlink6.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = transformToolMount_X_Endeffector.cast<SCALAR_T>();
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<SCALAR_T, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (SCALAR_T)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  return result;
}

template class perceptive_mpc::asArmKinematics<double>;
template class perceptive_mpc::asArmKinematics<CppAD::AD<CppAD::cg::CG<double>>>;