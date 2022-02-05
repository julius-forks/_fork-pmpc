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

#pragma once
#include <ocs2_core/Dimensions.h>
#include <perceptive_mpc/Definitions.h>

namespace perceptive_mpc {

inline ocs2::dynamic_vector_t interpolateBaseArmElipseTrajectory(const ocs2::scalar_array_t& timeTrajectory,
                                                        const ocs2::dynamic_vector_array_t& stateTrajectory, ocs2::scalar_t time) {
  ocs2::dynamic_vector_t reference((int)Definitions::REFERENCE_DIM);

  auto it = std::lower_bound(timeTrajectory.begin(), timeTrajectory.end(), time);
  int timeAIdx = it - timeTrajectory.begin() - 1;
  if (timeAIdx == -1) {
    reference = stateTrajectory[0].head((int)Definitions::REFERENCE_DIM);
    /*
        std::cerr << "t:" << t << ", t_start:" << desiredTimeTrajectory.front() << ", t_stop:" << desiredTimeTrajectory.back()
                  << std::endl << ", timeAIdx:" << timeAIdx << ", N:" << desiredTimeTrajectory.size() << ", t_idx:" <<
       desiredTimeTrajectory[timeAIdx] << std::endl << std::endl;
    */
  } else if (timeAIdx == timeTrajectory.size() - 1) {
    reference = stateTrajectory[timeAIdx].head((int)Definitions::REFERENCE_DIM);
    ;
    /*
        std::cerr << "t:" << t << ", t_start:" << desiredTimeTrajectory.front() << ", t_stop:" << desiredTimeTrajectory.back()
                  << std::endl << ", timeAIdx:" << timeAIdx << ", N:" << desiredTimeTrajectory.size() << ", t_idx:" <<
       desiredTimeTrajectory[timeAIdx] << std::endl << std::endl;
    */
  } else {
    double tau = (time - timeTrajectory[timeAIdx]) / (timeTrajectory[timeAIdx + 1] - timeTrajectory[timeAIdx]);

    const Eigen::Quaterniond ee_quatA(stateTrajectory[timeAIdx].segment<4>(0));//ee pose quat
    const Eigen::Quaterniond ee_quatB(stateTrajectory[timeAIdx + 1].segment<4>(0));

    const Eigen::Quaterniond base_quatA(stateTrajectory[timeAIdx].segment<4>((int) Definitions::POSE_DIM));//should be the quat of base pose
    const Eigen::Quaterniond base_quatB(stateTrajectory[timeAIdx + 1].segment<4>((int) Definitions::POSE_DIM));


    // interpolate the quaternions using slerp
    reference.segment<4>(0) = ee_quatA.slerp(tau, ee_quatB).coeffs();// ee quat
    reference.segment<4>((int) Definitions::POSE_DIM) = base_quatA.slerp(tau, base_quatB).coeffs();
    // interpolate cartesian space position linearly
    reference.segment<3>(4) = (1 - tau) * stateTrajectory[timeAIdx].segment<3>(4)+
                          tau * stateTrajectory[timeAIdx + 1].segment<3>(4);

    reference.segment<3>(11) = (1 - tau) * stateTrajectory[timeAIdx].segment<3>(11) +
                          tau * stateTrajectory[timeAIdx + 1].segment<3>(11);

    reference.segment<3>(14) = (1 - tau) * stateTrajectory[timeAIdx].segment<3>(14) +
                          tau * stateTrajectory[timeAIdx + 1].segment<3>(14);
    
    /*    std::cerr << "t:" << t << ", tau:" << tau << ", t_start:" << desiredTimeTrajectory.front() << ", t_stop:" <<
       desiredTimeTrajectory.back()
                  << std::endl << ", timeAIdx:" << timeAIdx << ", N:" << desiredTimeTrajectory.size() << ", t_idx:" <<
       desiredTimeTrajectory[timeAIdx] << std::endl
                  << "x_a: " << desiredStateTrajectory[timeAIdx].transpose() << std::endl
                  << "x_t: " << xNominal.transpose() << std::endl
                  << "x_b: " << desiredStateTrajectory[timeAIdx+1].transpose() << std::endl<< std::endl;*/
  }
  return reference;
}

}  // namespace perceptive_mpc
