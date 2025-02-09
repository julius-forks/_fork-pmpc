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

#include "armstone/AsPerceptiveMpcInterface.h"
#include <perceptive_mpc/costs/BaseAvoidanceCost.h>
#include <perceptive_mpc/costs/QuadraticBaseEETrackingCost.h>
#include <perceptive_mpc/costs/QuadraticEndeffectorTrackingCost.h>
#include <perceptive_mpc/costs/QuadraticBaseTrackingCost.h>
#include <perceptive_mpc/costs/BaseElipseCost.h>
#include <perceptive_mpc/costs/VoxbloxCost.h>
#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

#include <cmath>

namespace perceptive_mpc
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  AsPerceptiveMpcInterface::AsPerceptiveMpcInterface(const AsPerceptiveMpcInterfaceConfig &config)
      : voxbloxConfig_(config.voxbloxConfig), kinematicsInterface_(config.kinematicsInterface)
  {

    std::string taskFileName = config.taskFileName;

    std::string packagePath = ros::package::getPath("perceptive_mpc");

    taskFile_ = packagePath + "/config/" + taskFileName;
    std::cerr << "Loading task file: " << taskFile_ << std::endl;

    libraryFolder_ = packagePath + "/auto_generated/" + taskFileName;
    std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

    // load setting from loading file
    loadSettings(taskFile_);

    // MPC
    resetMpc();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void AsPerceptiveMpcInterface::loadSettings(const std::string &taskFile)
  {
    /*
   * Default initial condition
   */
    ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

    /*
   * SLQ-MPC settings
   */
    slqSettings_.loadSettings(taskFile);
    mpcSettings_.loadSettings(taskFile);

    /*
   * Model settings
   */
    modelSettings_.loadSettings(taskFile);

    /*
   * Dynamics
   */
    systemDynamicsPtr_.reset(new SystemDynamics());

    systemDynamicsPtr_->initialize("base_arm_kinematics", libraryFolder_, modelSettings_.recompileLibraries_);

    /*
   * Cost functions
   */
    using WeightedCost = cost_linear_combination_t::WeightedCost;
    std::vector<WeightedCost> weightedCostFunctions;

    /*
   * EE Only
   */
    bool useEECost = false;
    ocs2::loadData::loadCppDataType(taskFile, "ee_tracking_task.use", useEECost);
    std::cerr << "useEECost:       \n"
              << useEECost << std::endl;
    if (useEECost)
    {
      // task space tracking cost
      QuadraticEndeffectorTrackingCostConfig config;
      ocs2::loadData::loadEigenMatrix(taskFile, "ee_tracking_task.Q", config.ee_Q);
      ocs2::loadData::loadEigenMatrix(taskFile, "ee_tracking_task.R", config.ee_R);
      ocs2::loadData::loadEigenMatrix(taskFile, "ee_tracking_task.Q_final", config.ee_QFinal);
      config.kinematics = kinematicsInterface_;

      std::cerr << "Q:       \n"
                << config.ee_Q << std::endl;
      std::cerr << "R:       \n"
                << config.ee_R << std::endl;
      std::cerr << "Q_final: \n"
                << config.ee_QFinal << std::endl;

      std::shared_ptr<QuadraticEndeffectorTrackingCost> ee_Cost(new QuadraticEndeffectorTrackingCost(config));
      ee_Cost->initialize("ee_quadratic_cost", libraryFolder_, modelSettings_.recompileLibraries_);

      weightedCostFunctions.push_back(std::make_pair(1, ee_Cost));
    }

    /*
    * Base Only
    */

    bool useBaseCost = false;
    ocs2::loadData::loadCppDataType(taskFile, "base_tracking_task.use", useBaseCost);
    std::cerr << "useBaseCost:       \n"
              << useBaseCost << std::endl;
    if (useBaseCost)
    {
      // task space tracking cost
      QuadraticBaseTrackingCostConfig config;
      ocs2::loadData::loadEigenMatrix(taskFile, "base_tracking_task.Q", config.base_Q);
      ocs2::loadData::loadEigenMatrix(taskFile, "base_tracking_task.R", config.base_R);
      ocs2::loadData::loadEigenMatrix(taskFile, "base_tracking_task.Q_final", config.base_QFinal);
      config.kinematics = kinematicsInterface_;

      std::cerr << "Q:       \n"
                << config.base_Q << std::endl;
      std::cerr << "R:       \n"
                << config.base_R << std::endl;
      std::cerr << "Q_final: \n"
                << config.base_QFinal << std::endl;

      std::shared_ptr<QuadraticBaseTrackingCost> base_Cost(new QuadraticBaseTrackingCost(config));
      base_Cost->initialize("base_quadratic_cost", libraryFolder_, modelSettings_.recompileLibraries_);

      weightedCostFunctions.push_back(std::make_pair(1, base_Cost));
    }
    /*
    * Base EE combined 
    */

    bool useBaseEECosts = false;
    ocs2::loadData::loadCppDataType(taskFile, "base_ee_tracking_task.use", useBaseEECosts);
    std::cerr << "useBaseEECosts:       \n"
              << useBaseEECosts << std::endl;
    if (useBaseEECosts)
    {
      // task space tracking cost
      QuadraticBaseEETrackingCostConfig config;
      ocs2::loadData::loadEigenMatrix(taskFile, "base_ee_tracking_task.Q", config.base_ee_Q);
      ocs2::loadData::loadEigenMatrix(taskFile, "base_ee_tracking_task.R", config.base_ee_R);
      ocs2::loadData::loadEigenMatrix(taskFile, "base_ee_tracking_task.Q_final", config.base_ee_QFinal);
      config.kinematics = kinematicsInterface_;
      std::cerr << "Q:       \n"
                << config.base_ee_Q << std::endl;
      std::cerr << "R:       \n"
                << config.base_ee_R << std::endl;
      std::cerr << "Q_final: \n"
                << config.base_ee_QFinal << std::endl;

      std::shared_ptr<QuadraticBaseEETrackingCost> base_ee_Cost(new QuadraticBaseEETrackingCost(config));
      base_ee_Cost->initialize("base_ee_quadratic_cost", libraryFolder_, modelSettings_.recompileLibraries_);
      std::cerr << "useBaseEECosts: 1" << std::endl;
      weightedCostFunctions.push_back(std::make_pair(1, base_ee_Cost));
      std::cerr << "useBaseEECosts: Loaded" << std::endl;
    }

    /*
    * Base Elipse
    */

    bool useBaseElipseCost = false;
    ocs2::loadData::loadCppDataType(taskFile, "base_elipse_cost.use", useBaseElipseCost);
    std::cerr << "useBaseElipseCost:       \n"
              << useBaseElipseCost << std::endl;

    if (useBaseElipseCost)
    {
      perceptive_mpc::BaseElipseCostConfig config;

      ocs2::loadData::loadCppDataType(taskFile, "base_elipse_cost.delta", config.delta);
      std::cerr << "BaseElipseCost.delta_:       \n"
                << config.delta << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "base_elipse_cost.mu", config.mu);
      std::cerr << "BaseElipseCost.mu:       \n"
                << config.mu << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "base_elipse_cost.sigma", config.sigma);
      std::cerr << "BaseElipseCost.sigma:       \n"
                << config.sigma << std::endl;      

      config.kinematics = kinematicsInterface_;
      std::shared_ptr<BaseElipseCost> baseElipseCost(new BaseElipseCost(config));
      baseElipseCost->initialize("base_elipse_cost", libraryFolder_, modelSettings_.recompileLibraries_);

      weightedCostFunctions.push_back(std::make_pair(1, baseElipseCost));
    }
    
    
    bool useObstacleCost = false;
    ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.useObstacleCost", useObstacleCost);
    std::cerr << "useObstacleCost:       \n"
              << useObstacleCost << std::endl;

    if (useObstacleCost)
    {
      perceptive_mpc::BaseAvoidanceCostConfig config;

      ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.delta", config.delta);
      std::cerr << "obstacleCost.delta_:       \n"
                << config.delta << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.mu", config.mu);
      std::cerr << "obstacleCost.mu:       \n"
                << config.mu << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.sigma", config.sigma);
      std::cerr << "obstacleCost.sigma:       \n"
                << config.sigma << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.widthX", config.widthX);
      std::cerr << "obstacleCost.widthX:       \n"
                << config.widthX << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.widthY", config.widthY);
      std::cerr << "obstacleCost.widthY:       \n"
                << config.widthY << std::endl;

      ocs2::loadData::loadCppDataType(taskFile, "obstacle_cost.reach", config.reach);
      std::cerr << "obstacleCost.reach:       \n"
                << config.reach << std::endl;

      config.kinematics = kinematicsInterface_;
      std::shared_ptr<BaseAvoidanceCost> baseAvoidanceCost(new BaseAvoidanceCost(config));
      baseAvoidanceCost->initialize("base_avoidance_cost", libraryFolder_, modelSettings_.recompileLibraries_);

      weightedCostFunctions.push_back(std::make_pair(1, baseAvoidanceCost));
    }

    if (voxbloxConfig_)
    {
      ocs2::loadData::loadCppDataType(taskFile, "voxblox_cost.mu", voxbloxConfig_->mu);
      std::cerr << "voxblox_cost.mu:       \n"
                << voxbloxConfig_->mu << std::endl;
      ocs2::loadData::loadCppDataType(taskFile, "voxblox_cost.delta", voxbloxConfig_->delta);
      std::cerr << "voxblox_cost.delta:       \n"
                << voxbloxConfig_->delta << std::endl;
      ocs2::loadData::loadCppDataType(taskFile, "voxblox_cost.max_distance", voxbloxConfig_->maxDistance);
      std::cerr << "voxblox_cost.max_distance:       \n"
                << voxbloxConfig_->maxDistance << std::endl;
      std::shared_ptr<VoxbloxCost> voxbloxCost(new VoxbloxCost(*voxbloxConfig_));
      weightedCostFunctions.push_back(std::make_pair(1, voxbloxCost));
    }

    costPtr_.reset(new cost_linear_combination_t(weightedCostFunctions));

    Eigen::VectorXd lowerLimits((int)Definitions::ARM_STATE_DIM_);
    ocs2::loadData::loadEigenMatrix(taskFile, "limits.lower", lowerLimits);
    Eigen::VectorXd upperLimits((int)Definitions::ARM_STATE_DIM_);
    ocs2::loadData::loadEigenMatrix(taskFile, "limits.upper", upperLimits);
    Eigen::VectorXd velocityLimits((int)Definitions::INPUT_DIM_);
    ocs2::loadData::loadEigenMatrix(taskFile, "limits.velocity", velocityLimits);
    std::cerr << "lowerLimits:       \n"
              << lowerLimits << std::endl;
    std::cerr << "upperLimits:       \n"
              << upperLimits << std::endl;
    std::cerr << "velocityLimits:    \n"
              << velocityLimits << std::endl;

    bool useJointSpaceConstraints = false;
    ocs2::loadData::loadCppDataType(taskFile, "limits.enforce_limits", useJointSpaceConstraints);
    std::cerr << "useJointSpaceConstraints:       \n"
              << useJointSpaceConstraints << std::endl;
    if (useJointSpaceConstraints)
    {
      double positionMpcMarginDeg = 0;
      ocs2::loadData::loadCppDataType(taskFile, "limits.position_mpc_margin_deg", positionMpcMarginDeg);
      std::cerr << "positionMpcMarginDeg:       \n"
                << positionMpcMarginDeg << std::endl;
      double positionMpcMarginRad = positionMpcMarginDeg / 180 * M_PI;

      setupConstraints(lowerLimits, upperLimits, velocityLimits, positionMpcMarginRad);
    }
    else
    {
      constraintPtr_.reset(new constraint_t());
    }

    operatingPointPtr_.reset(new OperatingPoint());

    /*
   * Rollout
   */
    ocs2::Rollout_Settings rolloutSettings;
    rolloutSettings.loadSettings(taskFile, "slq.rollout");
    timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*systemDynamicsPtr_, rolloutSettings));

    /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
    dim_t::scalar_t timeHorizon;
    ocs2::loadData::loadPartitioningTimes(taskFile, timeHorizon, numPartitions_, partitioningTimes_, true);
  }
  void AsPerceptiveMpcInterface::setupConstraints(const Eigen::VectorXd &lowerLimits, const Eigen::VectorXd &upperLimits,
                                                  const Eigen::VectorXd &velocityLimits, double positionMpcMarginRad)
  {
    std::unique_ptr<linear_constraint_t> linearConstraintPtr(new linear_constraint_t());

    linearConstraintPtr->numInequalityConstraint_ = 2 * Definitions::ARM_STATE_DIM_ + 2 * Definitions::INPUT_DIM_;
    Eigen::VectorXd h0Eigen(linearConstraintPtr->numInequalityConstraint_);
    h0Eigen << -1 * (lowerLimits + Eigen::VectorXd::Ones(Definitions::ARM_STATE_DIM_) * positionMpcMarginRad),
        1 * (upperLimits - Eigen::VectorXd::Ones(Definitions::ARM_STATE_DIM_) * positionMpcMarginRad), velocityLimits, velocityLimits;
    linearConstraintPtr->h0_.resize(linearConstraintPtr->numInequalityConstraint_);
    for (int i = 0; i < linearConstraintPtr->numInequalityConstraint_; i++)
    {
      linearConstraintPtr->h0_[i] = h0Eigen[i];
    }

    linearConstraintPtr->dhdx_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::state_vector_t::Zero());
    const int armStateOffset = STATE_DIM_ - ARM_STATE_DIM_;

    for (int i = 0; i < ARM_STATE_DIM_; i++)
    {
      linearConstraintPtr->dhdx_[i][armStateOffset + i] = 1;
      linearConstraintPtr->dhdx_[i + ARM_STATE_DIM_][armStateOffset + i] = -1;
    }

    linearConstraintPtr->dhdu_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::input_vector_t::Zero());
    const int velocityConstraintOffset = 2 * ARM_STATE_DIM_;
    for (int i = 0; i < INPUT_DIM_; i++)
    {
      linearConstraintPtr->dhdu_[velocityConstraintOffset + i][i] = 1;
      linearConstraintPtr->dhdu_[velocityConstraintOffset + INPUT_DIM_ + i][i] = -1;
    }

    linearConstraintPtr->ddhdxdx_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::state_matrix_t::Zero());
    linearConstraintPtr->ddhdudu_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::input_matrix_t::Zero());
    linearConstraintPtr->ddhdudx_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::input_state_matrix_t::Zero());

    for (int i = 0; i < linearConstraintPtr->numInequalityConstraint_; i++)
    {
      std::cerr << "Constraint " << i << ": " << linearConstraintPtr->h0_[i] << " + [" << linearConstraintPtr->dhdx_[i].transpose()
                << "] * x + [" << linearConstraintPtr->dhdu_[i].transpose() << "] * u >= 0" << std::endl;
    }

    constraintPtr_ = move(linearConstraintPtr);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void AsPerceptiveMpcInterface::resetMpc()
  {
    mpcPtr_.reset(new mpc_t(timeTriggeredRolloutPtr_.get(), systemDynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                            operatingPointPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  AsPerceptiveMpcInterface::mpc_t &AsPerceptiveMpcInterface::getMpc()
  {
    return *mpcPtr_;
  }

} // namespace perceptive_mpc