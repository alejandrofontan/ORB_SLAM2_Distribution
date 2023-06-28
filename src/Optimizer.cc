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

#include "Optimizer.h"


#include<Eigen/StdVector>

#include "Converter.h"
#include "DistributionFitter.h"

#include<mutex>

namespace ORB_SLAM2
{

OptimizerParameters Optimizer::parameters{};
#ifdef COMPILED_DEBUG
    //vector<double> Optimizer::mahalanobisDistancesToSave{};
    vector<double> inlierThreshold{};
#endif

void Optimizer::GlobalBundleAdjustment(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<Keyframe> keyframes = pMap->GetAllKeyFrames();
    vector<MapPt> mapPoints = pMap->GetAllMapPoints();
    BundleAdjustment(keyframes,mapPoints,nIterations,pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::GlobalRobustBundleAdjustment(Map* pMap)
{
        vector<Keyframe> keyframes = pMap->GetAllKeyFrames();
        vector<MapPt> mapPoints = pMap->GetAllMapPoints();
        RobustBundleAdjustment(keyframes,mapPoints);
}

void Optimizer::RobustBundleAdjustment(const vector<Keyframe> &keyframes, const vector<MapPt> &mapPoints, const unsigned long nLoopKF)
{

    // Set optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices
    KeyframeId maxKeyId{0};
    for(const auto& keyframe: keyframes)
    {
        if(keyframe->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(keyframe->GetPose()));
        vSE3->setId(keyframe->mnId);
        vSE3->setFixed(keyframe->mnId==0);
        optimizer.addVertex(vSE3);
        if(keyframe->mnId > maxKeyId)
            maxKeyId = keyframe->mnId;
    }

    // Set MapPoint vertices
    const float thHuber2D = parameters.deltaMono;
    const float thHuber3D = parameters.deltaStereo;
    vector<bool> mapPtsNotInclude{};
    mapPtsNotInclude.resize(mapPoints.size());

    int jMapPt{0};
    vector<g2o::EdgeSE3ProjectXYZ*>edgesMono{};
    vector<g2o::EdgeStereoSE3ProjectXYZ*>edgesStereo{};
    vector<Keyframe> keyframesMono{},keyframesStereo{};
    vector<MapPt> mapPointsMono{},mapPointsStereo{};
    for(const auto& mapPt: mapPoints)
    {
        if(mapPt->isBad()){
            ++jMapPt;
            continue;
        }

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(mapPt->GetWorldPos()));
        const int id = mapPt->mnId + maxKeyId + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        //mapPt->activateAllObservations();
        const map<KeyframeId , Observation> observations = mapPt->GetActiveObservations();

        //Set edges
        int nEdges = 0;
        for(auto& obs: observations)
        {
            Keyframe keyframe = obs.second.projKeyframe;
            if(keyframe->isBad() || keyframe->mnId > maxKeyId){
                continue;
            }

            nEdges++;

            const cv::KeyPoint &kpUn = keyframe->mvKeysUn[obs.second.projIndex];
            const float &invSigma2 = keyframe->mvInvLevelSigma2[kpUn.octave];

            if(keyframe->mvuRight[obs.second.projIndex] < 0)
            {
                Eigen::Matrix<double,2,1> obs2D;
                obs2D << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                setVertex(e,&optimizer, obs2D, id, keyframe->mnId);

                Eigen::Matrix2d Info = Eigen::Matrix2d::Identity()*invSigma2;
                e->setInformation(Info);

                setEdgeRobustKernel(e, thHuber2D);
                setEdgeMonoIntrinsics(e, keyframe);
                optimizer.addEdge(e);

                edgesMono.push_back(e);
                keyframesMono.push_back(keyframe);
                mapPointsMono.push_back(mapPt);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs3D;
                const float kp_ur = keyframe->mvuRight[obs.second.projIndex];
                obs3D << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
                setVertex(e,&optimizer, obs3D, id, keyframe->mnId);

                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                setEdgeRobustKernel(e, thHuber3D);
                setEdgeStereoIntrinsics(e, keyframe);
                optimizer.addEdge(e);

                edgesStereo.push_back(e);
                keyframesStereo.push_back(keyframe);
                mapPointsStereo.push_back(mapPt);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            mapPtsNotInclude[jMapPt] = true;
        }
        else
        {
            mapPtsNotInclude[jMapPt] = false;
        }
        ++jMapPt;
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(parameters.globalRobustBundleAdjustment.optimizerItsCoarse);

    vector<double> mahalanobisDistancesMono{};
    for(const auto& edge: edgesMono){
        edge->computeError();
        mahalanobisDistancesMono.push_back(edge->chi2());
    }
    vector<double> mahalanobisDistancesStereo{};
    for(const auto& edge: edgesStereo){
        edge->computeError();
        mahalanobisDistancesStereo.push_back(edge->chi2());
    }

    std::vector<double> mahalanobisDistances(mahalanobisDistancesMono.begin(), mahalanobisDistancesMono.end());
    mahalanobisDistances.insert(mahalanobisDistances.end(), mahalanobisDistancesStereo.begin(), mahalanobisDistancesStereo.end());

    double inlierThresholdMono{parameters.inlierThresholdMono};
    double inlierThresholdStereo{parameters.inlierThresholdStereo};

    if(DIST_FITTER::DistributionFitter::distributionType == DIST_FITTER::DistributionFitter::DistributionType::BURR){
        double k{parameters.localBundleAdjustment.k_burr};
        double alpha{parameters.localBundleAdjustment.alpha_burr};
        double beta{parameters.localBundleAdjustment.beta_burr};
        DIST_FITTER::DistributionFitter::FitBurr(mahalanobisDistances,k,alpha,beta);
        inlierThresholdMono = DIST_FITTER::DistributionFitter::Burr_icdf(parameters.inlierProbability, k, alpha, beta, inlierThresholdMono);
    }

    if(DIST_FITTER::DistributionFitter::distributionType == DIST_FITTER::DistributionFitter::DistributionType::LOGNORMAL){
        double mu{parameters.localBundleAdjustment.mu_lognormal}, sigma{parameters.localBundleAdjustment.sigma_lognormal};
        DIST_FITTER::DistributionFitter::FitLogNormal(mahalanobisDistances,
                                                  parameters.localBundleAdjustment.mu_lognormal,
                                                  parameters.localBundleAdjustment.sigma_lognormal);
        inlierThresholdMono = DIST_FITTER::DistributionFitter::Lognormal_icdf(parameters.inlierProbability,
                                                                      parameters.localBundleAdjustment.mu_lognormal,
                                                                      parameters.localBundleAdjustment.sigma_lognormal);
    }

    vector<bool> isInlierMono =  DIST_FITTER::DistributionFitter::GetInliers(mahalanobisDistancesMono,inlierThresholdMono);
    vector<bool> isInlierStereo =  DIST_FITTER::DistributionFitter::GetInliers(mahalanobisDistancesStereo,inlierThresholdStereo);

    // Check inlier observations
    setInliers(edgesMono, isInlierMono);
    setInliers(edgesStereo, isInlierStereo);
    //setEdgesRobustKernel(edgesMono, float(sqrt(inlierThresholdMono)));
    //setEdgesRobustKernel(edgesStereo, float(sqrt(inlierThresholdStereo)));

    deactivateRobustKernel(edgesMono);
    deactivateRobustKernel(edgesStereo);

    // Optimize again without the outliers
    ResetOptimizerVariables(keyframes,mapPoints,optimizer,maxKeyId,mapPtsNotInclude);
    optimizer.initializeOptimization();
    optimizer.optimize(parameters.globalRobustBundleAdjustment.optimizerItsFine);

    //Keyframes
    for(size_t i=0; i < keyframes.size(); i++)
    {
        KeyFrame* pKF = keyframes[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i< mapPoints.size(); i++)
    {
        if(mapPtsNotInclude[i])
            continue;

        MapPoint* pMP = mapPoints[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKeyId+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
}

void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = parameters.deltaMono;
    const float thHuber3D = parameters.deltaStereo;

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        //pMP->activateAllObservations();
        const map<KeyframeId , Observation> observations = pMP->GetActiveObservations();

        int nEdges = 0;
        //SET EDGES
        for(auto& obs: observations)
        {

            Keyframe pKF = obs.second.projKeyframe;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[obs.second.projIndex];

            if(pKF->mvuRight[obs.second.projIndex] < 0)
            {
                Eigen::Matrix<double,2,1> obs2D;
                obs2D << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs2D);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs2D;
                const float kp_ur = pKF->mvuRight[obs.second.projIndex];
                obs2D << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs2D);

                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = parameters.deltaMono;
    const float deltaStereo = parameters.deltaStereo;

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences < parameters.poseOptimization.nInitialCorrespondences)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    std::vector<float> chi2Mono(parameters.poseOptimization.its, parameters.chi2_2dof);
    std::vector<float> chi2Stereo(parameters.poseOptimization.its, parameters.chi2_3dof);
    std::vector<int> its(parameters.poseOptimization.its, parameters.poseOptimization.optimizerIts);

    int nBad=0;
    for(size_t it = 0; it < parameters.poseOptimization.its; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size() < parameters.poseOptimization.minimumNumberOfEdges)
            break;
    }    

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    map<int,KeyFrame*> fixedKeyframes{};
    for(auto& mapPt : lLocalMapPoints)
    {
        mapPt->activateAllObservations();
        map<KeyframeId , Observation> observations = mapPt->GetActiveObservations();
        for(auto& obs :observations)
        {
            Keyframe keyframe_i = obs.second.projKeyframe;

            if(keyframe_i->mnBALocalForKF!=pKF->mnId && keyframe_i->mnBAFixedForKF!=pKF->mnId)
            {
                keyframe_i->mnBAFixedForKF=pKF->mnId;
                if(!keyframe_i->isBad())
                    lFixedCameras.push_back(keyframe_i);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = parameters.deltaMono;
    const float thHuberStereo = parameters.deltaStereo;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        pMP->activateAllObservations();
        map<long unsigned int, Observation> observations = pMP->GetActiveObservations();

        //Set edges
        for(auto& obs: observations)
        {
            KeyFrame* pKFi = obs.second.projKeyframe;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[obs.second.projIndex];

                // Monocular observation
                if(pKFi->mvuRight[obs.second.projIndex]<0)
                {
                    Eigen::Matrix<double,2,1> obs2D;
                    obs2D << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs2D);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs2D;
                    const float kp_ur = pKFi->mvuRight[obs.second.projIndex];
                    obs2D << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs2D);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(parameters.localBundleAdjustment.optimizerItsCoarse);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2() > parameters.chi2_2dof || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2() > parameters.chi2_3dof || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(parameters.localBundleAdjustment.optimizerItsFine);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2() > parameters.chi2_2dof || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2() > parameters.chi2_3dof || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            auto obs = pMPi->GetObservation(pKFi->mnId);
            if(obs)
                obs->setActive(false);
            //pKFi->EraseMapPointMatch(pMPi);
            //pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(auto& pKF:lLocalKeyFrames)
    {
        //KeyFrame* pKF = lit.se;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}

void Optimizer::RobustLocalBundleAdjustment(Keyframe& refKeyframe, bool *stopFlag, Map* map_)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<Keyframe> localKeyframes{};

    localKeyframes.push_back(refKeyframe);
    refKeyframe->mnBALocalForKF = refKeyframe->mnId;

    vector<Keyframe> neighbors = refKeyframe->GetVectorCovisibleKeyFrames();
    for(auto& neighbor:neighbors){
        neighbor->mnBALocalForKF = refKeyframe->mnId;
        if(!neighbor->isBad())
            localKeyframes.push_back(neighbor);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPt> localMapPoints{};
    for(auto& localKeyframe: localKeyframes){
        vector<MapPt> mapPoints = localKeyframe->GetMapPointMatches();
        for(auto& mapPt: mapPoints){
            if(mapPt){
                if(!mapPt->isBad()) {
                    if (mapPt->mnBALocalForKF != refKeyframe->mnId) {
                        localMapPoints.push_back(mapPt);
                        mapPt->mnBALocalForKF = refKeyframe->mnId;
                    }
                }
            }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<Keyframe> fixedCameras{};
    map<int,Keyframe> fixedKeyframes{};

    for(auto& mapPt : localMapPoints){
        mapPt->activateAllObservations();
        map<KeyframeId , Observation> observations = mapPt->GetActiveObservations();
        for(auto& obs :observations){
            Keyframe keyframe_i = obs.second.projKeyframe;
            if(keyframe_i->mnBALocalForKF != refKeyframe->mnId && keyframe_i->mnBAFixedForKF != refKeyframe->mnId){
                keyframe_i->mnBAFixedForKF = refKeyframe->mnId;
                if(!keyframe_i->isBad()){
                    fixedCameras.push_back(keyframe_i);
                }
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(stopFlag)
        optimizer.setForceStopFlag(stopFlag);


    // Set Local KeyFrame vertices
    KeyframeId maxKFid = 0;
    for(auto& localKeyframe: localKeyframes){
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(localKeyframe->GetPose()));
        vSE3->setId(localKeyframe->mnId);
        vSE3->setFixed(localKeyframe->mnId==0);
        optimizer.addVertex(vSE3);
        if(localKeyframe->mnId>maxKFid)
            maxKFid = localKeyframe->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(auto& fixedCamera: fixedCameras){
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(fixedCamera->GetPose()));
        vSE3->setId(fixedCamera->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(fixedCamera->mnId > maxKFid)
            maxKFid = fixedCamera->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (localKeyframes.size() + fixedCameras.size())*localMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> edgesMono;
    edgesMono.reserve(nExpectedSize);

    vector<Keyframe> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPt> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> edgesStereo;
    edgesStereo.reserve(nExpectedSize);

    vector<Keyframe> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPt> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    //const float thHuberMono = parameters.deltaMono;
    //const float thHuberStereo = parameters.deltaStereo;

    const float thHuberMono = sqrt(parameters.inlierThresholdMono);
    const float thHuberStereo = sqrt(parameters.inlierThresholdStereo);

    for(auto& mapPt: localMapPoints){
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(mapPt->GetWorldPos()));
        int id = mapPt->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        mapPt->activateAllObservations();
        map<long unsigned int, Observation> observations = mapPt->GetActiveObservations();

        //Set edges
        for(auto& obs: observations){
            Keyframe keyframe = obs.second.projKeyframe;

            if(!keyframe->isBad()){

                const cv::KeyPoint &kpUn = keyframe->mvKeysUn[obs.second.projIndex];

                // Monocular observation
                if(keyframe->mvuRight[obs.second.projIndex]<0){
                    Eigen::Matrix<double,2,1> obs2D;
                    obs2D << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    setVertex(e, &optimizer, obs2D,id,keyframe->mnId);

                    const float &invSigma2 = keyframe->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                    setEdgeRobustKernel(e, thHuberMono);
                    setEdgeMonoIntrinsics(e, keyframe);

                    optimizer.addEdge(e);
                    edgesMono.push_back(e);
                    vpEdgeKFMono.push_back(keyframe);
                    vpMapPointEdgeMono.push_back(mapPt);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs3D;
                    const float kp_ur = keyframe->mvuRight[obs.second.projIndex];
                    obs3D << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    setVertex(e, &optimizer, obs3D,id,keyframe->mnId);

                    const float &invSigma2 = keyframe->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    setEdgeRobustKernel(e, thHuberStereo);
                    setEdgeStereoIntrinsics(e, keyframe);

                    optimizer.addEdge(e);
                    edgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(keyframe);
                    vpMapPointEdgeStereo.push_back(mapPt);
                }
            }
        }
    }

    if(stopFlag)
        if(*stopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(parameters.localBundleAdjustment.optimizerItsCoarse);

    bool bDoMore= true;

    if(stopFlag)
        if(*stopFlag)
            bDoMore = false;

    double inlierThresholdMono = parameters.inlierThresholdMono;
    double inlierThresholdStereo = parameters.inlierThresholdStereo;
    if(bDoMore){
        vector<double> mahalanobisDistancesMono{};
        for(const auto& edge: edgesMono){
            edge->computeError();
            mahalanobisDistancesMono.push_back(edge->chi2());
        }
        vector<double> mahalanobisDistancesStereo{};
        for(const auto& edge: edgesStereo){
            edge->computeError();
            mahalanobisDistancesStereo.push_back(edge->chi2());
        }

        std::vector<double> mahalanobisDistances(mahalanobisDistancesMono.begin(), mahalanobisDistancesMono.end());
        mahalanobisDistances.insert(mahalanobisDistances.end(), mahalanobisDistancesStereo.begin(),
                                    mahalanobisDistancesStereo.end());

        if(DIST_FITTER::DistributionFitter::distributionType == DIST_FITTER::DistributionFitter::DistributionType::BURR) {

            DIST_FITTER::DistributionFitter::FitBurr(mahalanobisDistances,
                                                     parameters.localBundleAdjustment.k_burr,
                                                     parameters.localBundleAdjustment.alpha_burr,
                                                     parameters.localBundleAdjustment.beta_burr);

            inlierThresholdMono = DIST_FITTER::DistributionFitter::Burr_icdf(
                    parameters.inlierProbability,
                    parameters.localBundleAdjustment.k_burr,
                    parameters.localBundleAdjustment.alpha_burr,
                    parameters.localBundleAdjustment.beta_burr,
                    parameters.inlierThresholdMono);

            Optimizer::parameters.UpdateInlierThresholds(inlierThresholdMono,inlierThresholdStereo);
        }

        if(DIST_FITTER::DistributionFitter::distributionType == DIST_FITTER::DistributionFitter::DistributionType::LOGNORMAL) {
            DIST_FITTER::DistributionFitter::FitLogNormal(mahalanobisDistances,
                                                      parameters.localBundleAdjustment.mu_lognormal,
                                                      parameters.localBundleAdjustment.sigma_lognormal);
            inlierThresholdMono = DIST_FITTER::DistributionFitter::Lognormal_icdf(parameters.inlierProbability,
                                                                            parameters.localBundleAdjustment.mu_lognormal,
                                                                            parameters.localBundleAdjustment.sigma_lognormal);

            Optimizer::parameters.UpdateInlierThresholds(inlierThresholdMono,inlierThresholdStereo);
        }

        vector<bool> isInlierMono =  DIST_FITTER::DistributionFitter::GetInliers(mahalanobisDistancesMono, parameters.inlierThresholdMono);
        vector<bool> isInlierStereo =  DIST_FITTER::DistributionFitter::GetInliers(mahalanobisDistancesStereo, parameters.inlierThresholdStereo);

#ifdef COMPILED_DEBUG
        Optimizer::inlierThreshold.push_back(inlierThresholdMono);
#endif

        // Check inlier observations
        setInliers(edgesMono, isInlierMono);
        setInliers(edgesStereo, isInlierStereo);
        //setEdgesRobustKernel(edgesMono, float(sqrt(burrThreshold)));
        //setEdgesRobustKernel(edgesStereo, float(sqrt(burrThreshold)));

        deactivateRobustKernel(edgesMono);
        deactivateRobustKernel(edgesStereo);

        // Optimize again without the outliers
        ResetOptimizerVariables(localKeyframes, localMapPoints, optimizer, maxKFid);
        optimizer.initializeOptimization(0);
        optimizer.optimize(parameters.localBundleAdjustment.optimizerItsFine);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(edgesMono.size() + edgesStereo.size());

    vector<double> mahalanobisDistancesMono{};
    for(const auto& edge: edgesMono){
        edge->computeError();
        mahalanobisDistancesMono.push_back(edge->chi2());
    }
    vector<double> mahalanobisDistancesStereo{};
    for(const auto& edge: edgesStereo){
        edge->computeError();
        mahalanobisDistancesStereo.push_back(edge->chi2());
    }

    std::vector<double> mahalanobisDistances(mahalanobisDistancesMono.begin(), mahalanobisDistancesMono.end());
    mahalanobisDistances.insert(mahalanobisDistances.end(), mahalanobisDistancesStereo.begin(), mahalanobisDistancesStereo.end());

    vector<bool> isInlierMono =  DIST_FITTER::DistributionFitter::GetInliers(mahalanobisDistancesMono,parameters.chi2_2dof);
    vector<bool> isInlierStereo =  DIST_FITTER::DistributionFitter::GetInliers(mahalanobisDistancesStereo,parameters.chi2_3dof);

    // Check inlier observations
    for(size_t i=0, iend = edgesMono.size(); i<iend;i++){
        MapPt pMP = vpMapPointEdgeMono[i];
        if(pMP->isBad())
            continue;

        if(!isInlierMono[i]){
            Keyframe pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend = edgesStereo.size(); i<iend;i++){
        MapPt pMP = vpMapPointEdgeStereo[i];
        if(pMP->isBad())
            continue;

        if(!isInlierStereo[i]){
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(map_->mMutexMapUpdate);

    if(!vToErase.empty()){
        for(size_t i=0;i<vToErase.size();i++){
            Keyframe pKFi = vToErase[i].first;
            MapPt pMPi = vToErase[i].second;
            auto obs = pMPi->GetObservation(pKFi->mnId);
            if(obs)
                obs->setActive(false);
            //pKFi->EraseMapPointMatch(pMPi);
            //pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(auto& pKF:localKeyframes){
        //KeyFrame* pKF = lit.se;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit = localMapPoints.begin(), lend = localMapPoints.end(); lit!=lend; lit++){
        MapPt pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}

void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       map<KeyframeId,LoopConnections> &loopConnections,
                                       const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(parameters.optimizeEssentialGraph.solverLambdaInit);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = parameters.optimizeEssentialGraph.minFeat;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF->mnId);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second.second;
            VSim3->setEstimate(it->second.second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(auto& mit : loopConnections)
    {
        KeyFrame* pKF = mit.second.keyframe;
        const long unsigned int nIDi = pKF->mnId;
        const map<KeyframeId, Keyframe> &spConnections = mit.second.connections;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(auto& sit : spConnections)
        {
            const long unsigned int nIDj = sit.first;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(sit.second)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF->mnId);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second.second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF->mnId);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second.second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const map<KeyframeId , Keyframe> sLoopEdges = pKF->GetLoopEdges();
        for(auto& pLKFTMp: sLoopEdges)
        {
            Keyframe pLKF = pLKFTMp.second;
            if(pLKF->mnId < pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF->mnId);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second.second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn->mnId))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn->mnId);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second.second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(parameters.optimizeEssentialGraph.optimizerIts);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
        ORB_SLAM2::KeyFrame::slamGraph->correctFramesScale((1./s), pKFi->mnFrameId);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(parameters.optimizeSim3.th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(parameters.optimizeSim3.optimizerIts);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2() > parameters.optimizeSim3.th2 || e21->chi2() > parameters.optimizeSim3.th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations = parameters.optimizeSim3.nMoreIterationsHigh;
    else
        nMoreIterations = parameters.optimizeSim3.nMoreIterationsLow;

    if(nCorrespondences-nBad < parameters.optimizeSim3.nBad)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2() > parameters.optimizeSim3.th2 || e21->chi2() > parameters.optimizeSim3.th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

std::ostream &operator<<(std::ostream &outstream, const OptimizerParameters &parameters) {
    cout << "\nOptimizer parameters : "<< endl;

    outstream << "- chi2_2dof: "        << parameters.chi2_2dof << endl;
    outstream << "- chi2_3dof: "        << parameters.chi2_3dof << endl;
    outstream << "- deltaMono: "        << parameters.deltaMono << endl;
    outstream << "- deltaStereo: "      << parameters.deltaStereo << endl;
    outstream << "- inlierProbability : "  << parameters.inlierProbability << endl;

    outstream << "- PoseOptimization : "<< endl;
    outstream << "    - nInitialCorrespondences: " << parameters.poseOptimization.nInitialCorrespondences << endl;
    outstream << "    - optimizer_its: "           << parameters.poseOptimization.optimizerIts << endl;
    outstream << "    - its : "                    << parameters.poseOptimization.its << endl;
    outstream << "    - minimumNumberOfEdges : "   << parameters.poseOptimization.minimumNumberOfEdges << endl;

    outstream << "- Local Bundle Adjustment : " << endl;
    outstream << "    - optimizerItsCoarse: "   << parameters.localBundleAdjustment.optimizerItsCoarse << endl;
    outstream << "    - optimizerItsFine : "    << parameters.localBundleAdjustment.optimizerItsFine << endl;

    outstream << "- Optimize Essential Graph : " << endl;
    outstream << "    - solverLambdaInit: "      << parameters.optimizeEssentialGraph.solverLambdaInit << endl;
    outstream << "    - minFeat : "              << parameters.optimizeEssentialGraph.minFeat << endl;
    outstream << "    - optimizerIts : "         << parameters.optimizeEssentialGraph.optimizerIts << endl;

    outstream << "- Optimize Sim3 : "<< endl;
    outstream << "    - optimizerIts: "          << parameters.optimizeSim3.optimizerIts << endl;
    outstream << "    - nMoreIterationsHigh : "  << parameters.optimizeSim3.nMoreIterationsHigh << endl;
    outstream << "    - nMoreIterationsLow : "   << parameters.optimizeSim3.nMoreIterationsLow << endl;
    outstream << "    - nBad : "                 << parameters.optimizeSim3.nBad << endl;
    outstream << "    - th2 : "                  << parameters.optimizeSim3.th2 << endl;

    outstream << "- Global Robust Bundle Adjustment : "<< endl;
    outstream << "    - optimizerItsCoarse: " << parameters.globalRobustBundleAdjustment.optimizerItsCoarse << endl;
    outstream << "    - optimizerItsFine : "  << parameters.globalRobustBundleAdjustment.optimizerItsFine << endl;

    return outstream;
}

template <typename Edge_, typename Frame_>
void Optimizer::setEdgeMonoIntrinsics(Edge_* e, Frame_ frame ){
    e->fx = frame->fx;
    e->fy = frame->fy;
    e->cx = frame->cx;
    e->cy = frame->cy;
}

template <typename Edge_, typename Frame_>
void Optimizer::setEdgeStereoIntrinsics(Edge_* e, Frame_ frame ){
    setEdgeMonoIntrinsics(e, frame);
    e->bf = frame->mbf;
}

template <typename Edge_>
void Optimizer::setEdgeRobustKernel(Edge_* e, const float& thHuber){
    auto* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber);
}

template <typename Edge_>
void Optimizer::setEdgesRobustKernel(vector<Edge_*>& edges, const float& thHuber){
    for(auto& e: edges){
        auto* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber);
    }
}

template <typename Edge_, typename Optimizer_, typename obs_>
void Optimizer::setVertex(Edge_* e, Optimizer_* optimizer,
                          const obs_& obs,
                          const int& mapPtId, const KeyframeId& keyframeId){
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(mapPtId)));
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(keyframeId)));
    e->setMeasurement(obs);
}

template <typename Edge_>
void Optimizer::setInliers(vector<Edge_*>& edges, const vector<bool>& isInlier){
    for(size_t iEdge{0}; iEdge < edges.size(); iEdge++)
    {
        Edge_* e = edges[iEdge];
        if(!isInlier[iEdge])
            e->setLevel(1);
        //e->setLevel(!isInlier[iEdge]);
    }
}

template <typename Edge_>
void Optimizer::deactivateRobustKernel(vector<Edge_*>& edges){
    for(auto& e: edges)
        e->setRobustKernel(nullptr);
}

template <typename Optimizer_>
void Optimizer::ResetOptimizerVariables(const list<Keyframe>& keyframes, const list<MapPt>& mapPoints,
                                        Optimizer_& optimizer,  const KeyframeId& maxKFid){
    //Keyframes
    for(auto& pKF:keyframes){
        auto* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
    }

    //Points
    for(auto& pMP: mapPoints){
        auto* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos().clone()));
    }
}

template <typename Optimizer_>
void Optimizer::ResetOptimizerVariables(const vector<Keyframe>& keyframes, const vector<MapPt>& mapPoints,
                                        Optimizer_& optimizer,  const KeyframeId& maxKFid,
                                        const vector<bool>& mapPtsNotInclude){
    //Keyframes
    for(auto& keyframe:keyframes){
        if(keyframe->isBad())
            continue;
        auto* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(keyframe->mnId));
        vSE3->setEstimate(Converter::toSE3Quat(keyframe->GetPose()));
    }

    //Points
    for(size_t iMapPt{0}; iMapPt < mapPoints.size(); iMapPt++){
        if(mapPtsNotInclude[iMapPt])
            continue;

        MapPt mapPt = mapPoints[iMapPt];

        if(mapPt->isBad())
            continue;

        auto* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(mapPt->mnId + maxKFid + 1));
        vPoint->setEstimate(Converter::toVector3d(mapPt->GetWorldPos().clone()));
    }
}

} //namespace ORB_SLAM

