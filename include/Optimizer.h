/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;
class OptimizerParameters;

class Optimizer {
public:
    static OptimizerParameters parameters;
#ifdef COMPILED_DEBUG
    #ifdef COMPILED_ABLATION_GBA
    static vector<double> residuals_u, residuals_v;
    #endif
    static vector<double> inlierThreshold;
    static vector<double> outlierPercentage;
#endif
    static vector<double> newThresholds;
    static vector<double> outlierPerc_chi2;
    static vector<double> outlierPerc_p;

    void static BundleAdjustment(const std::vector<Keyframe> &vpKF, const std::vector<MapPt> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag = nullptr, const unsigned long nLoopKF = 0,
                                 const bool bRobust = true);

    void static RobustBundleAdjustment(const std::vector<Keyframe> &vpKF, const std::vector<MapPt> &vpMP,
                                       const unsigned long nLoopKF = 0);

    void static GlobalBundleAdjustment(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                       const unsigned long nLoopKF = 0, const bool bRobust = true);

    void static GlobalRobustBundleAdjustment(Map *pMap);

    void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);
    void static RobustLocalBundleAdjustment(Keyframe& refKeyframe, bool *stopFlag, Map *map_);

    template <typename Optimizer_>
    void static ResetOptimizerVariables(const list<Keyframe>& keyframes, const list<MapPt>& mapPoints,
                                        Optimizer_& optimizer,
                                        const KeyframeId& maxKFid);
    template <typename Optimizer_>
    void static ResetOptimizerVariables(const vector<Keyframe>& keyframes, const vector<MapPt>& mapPoints,
                                        Optimizer_& optimizer,
                                        const KeyframeId& maxKFid,
                                        const vector<bool>& mapPtsNotInclude);

    int static PoseOptimization(Frame *pFrame);
    void static SetAllObservationsAsInliers(const vector<size_t>& indexes, Frame* frame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       map<KeyframeId, LoopConnections> &loopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const bool bFixScale);

    template <typename Edge_, typename Frame_>
    static void setEdgeMonoIntrinsics(Edge_* e, Frame_ frame );

    template <typename Edge_, typename Frame_>
    static void setEdgeStereoIntrinsics(Edge_* e, Frame_ frame );

    template <typename Edge_>
    static void setEdgeRobustKernel(Edge_* e, const float& thHuber);

    template <typename Edge_>
    static void setEdgesRobustKernel(vector<Edge_*>& edges, const float& thHuber);

    template <typename Edge_, typename Optimizer_, typename obs_>
    static void setVertex(Edge_* e, Optimizer_* optimizer, const obs_& obs, const int& mapPtId, const KeyframeId& keyframeId);

    template <typename Edge_>
    static void setInliers(vector<Edge_*>& edges, const vector<bool>& isInlier);

    template <typename Edge_>
    static void deactivateRobustKernel(vector<Edge_*>& edges);

    template <typename Edge_>
    static void setGeneralizedGaussian(vector<Edge_*>& edges, const double& exponent);

};

class OptimizerParameters {

public:
    struct PoseOptimizationParameters{
        int nInitialCorrespondences{3};
        int optimizerIts{10};
        int its{4};
        int minimumNumberOfEdges{10};
        PoseOptimizationParameters() = default;
        PoseOptimizationParameters(const int& nInitialCorrespondences, const int& optimizerIts, const int& its, const int& minimumNumberOfEdges):
        nInitialCorrespondences(nInitialCorrespondences),optimizerIts(optimizerIts),its(its),minimumNumberOfEdges(minimumNumberOfEdges){};
    };

    struct LocalBundleAdjustmentParameters{
        int optimizerItsCoarse{5};
        int optimizerItsFine{10};

        double k_burr{1.0},alpha_burr{1.0}, beta_burr{1.0};
        double mu_lognormal{1.0},sigma_lognormal{1.0};

        LocalBundleAdjustmentParameters() = default;
        LocalBundleAdjustmentParameters(const int& optimizerItsCoarse, const int& optimizerItsFine):
        optimizerItsCoarse(optimizerItsCoarse),optimizerItsFine(optimizerItsFine){};
    };

    struct OptimizeEssentialGraph{
        double solverLambdaInit{1e-16};
        int minFeat{100};
        int optimizerIts{20};
        OptimizeEssentialGraph() = default;
        OptimizeEssentialGraph(const double& solverLambdaInit, const int& minFeat, const int& optimizerIts):
        solverLambdaInit(solverLambdaInit),minFeat(minFeat),optimizerIts(optimizerIts){};
    };

    struct OptimizeSim3{
        int optimizerIts{5};
        int nMoreIterationsHigh{10};
        int nMoreIterationsLow{5};
        int nBad{10};
        float th2{10.0};

        OptimizeSim3() = default;
        OptimizeSim3(const int& optimizerIts, const int& nMoreIterationsHigh, const int& nMoreIterationsLow, const int& nBad, const float& th2):
        optimizerIts(optimizerIts),nMoreIterationsHigh(nMoreIterationsHigh),
        nMoreIterationsLow(nMoreIterationsLow),nBad(nBad),th2(th2){};
    };

    struct GlobalRobustBundleAdjustment{
        int optimizerItsCoarse{100};
        int optimizerItsFine{100};
        bool estimateOutlierThreshold{false};
        bool useGeneralizedGaussian{false};
        bool useTStudent{false};

        GlobalRobustBundleAdjustment() = default;
        GlobalRobustBundleAdjustment(const int& optimizerItsCoarse, const int& optimizerItsFine):
                optimizerItsCoarse(optimizerItsCoarse),optimizerItsFine(optimizerItsFine){};
    };

private:
    friend std::ostream& operator<<(std::ostream& outstream, const OptimizerParameters& parameters);
    friend class Optimizer;

    float chi2_2dof{5.991}; // Chi2 , 2 dof, 95%
    float chi2_3dof{7.815}; // Chi2 , 3 dof, 95%
    float deltaMono{sqrtf(chi2_2dof)};
    float deltaStereo{sqrtf(chi2_3dof)};

public:
    double th2_2dof{5.991};
    double th2_3dof{7.815};
    double th_2dof{sqrt(th2_2dof)};
    double th_3dof{sqrt(th2_3dof)};
    double inlierProbability{0.85};
    double pExp{0.52};
    double exponent{2.0};

    PoseOptimizationParameters poseOptimization{};
    LocalBundleAdjustmentParameters localBundleAdjustment{};
    OptimizeEssentialGraph optimizeEssentialGraph{};
    OptimizeSim3 optimizeSim3{};
    GlobalRobustBundleAdjustment globalRobustBundleAdjustment{};

    VerbosityLevel verbosity{VerbosityLevel::MEDIUM};

public:
    void setParameters(
            const float& chi2_2dof_, const float& chi2_3dof_,
            const double& inlierProbability_,
            const PoseOptimizationParameters& poseOptimization_,
            const LocalBundleAdjustmentParameters& localBundleAdjustment_,
            const OptimizeEssentialGraph& optimizeEssentialGraph_,
            const OptimizeSim3& optimizeSim3_,
            const GlobalRobustBundleAdjustment& globalRobustBundleAdjustment_){

        chi2_2dof = chi2_2dof_;
        chi2_3dof = chi2_3dof_;
        deltaMono = sqrt(chi2_2dof);
        deltaStereo = sqrt(chi2_3dof);

        inlierProbability = inlierProbability_;

        poseOptimization = poseOptimization_;
        localBundleAdjustment = localBundleAdjustment_;
        optimizeEssentialGraph = optimizeEssentialGraph_;
        optimizeSim3 = optimizeSim3_;
        globalRobustBundleAdjustment = globalRobustBundleAdjustment_;
    }

    void updateChi2(const float& chi2_2dof_, const float& chi2_3dof_){
        chi2_2dof = chi2_2dof_;
        chi2_3dof = chi2_3dof_;
        deltaMono = sqrt(chi2_2dof);
        deltaStereo = sqrt(chi2_3dof);
    }

    void UpdateInlierProbability(const double& inlierProbability_){
        inlierProbability = inlierProbability_;
    }

    void UpdateInlierThresholds(const double& th2_2dof_, const double& th2_3dof_){
        th2_2dof = th2_2dof_;
        th2_3dof = th2_3dof_;
        th_2dof = sqrt(th2_2dof);
        th_3dof = sqrt(th2_3dof);
    }

    void UpdateExponent(const double& exponent_){
        exponent = exponent_;
    }
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
