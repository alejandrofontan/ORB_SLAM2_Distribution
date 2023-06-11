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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2 {

    LocalMapping::LocalMapping(Map *pMap, const float bMonocular) :
            mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
            mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true) {
    }

    void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) {
        mpLoopCloser = pLoopCloser;
    }

    void LocalMapping::SetTracker(Tracking *pTracker) {
        mpTracker = pTracker;
    }

    void LocalMapping::Run() {

        mbFinished = false;

        while (1) {
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // Check if there are keyframes in the queue
            if (CheckNewKeyFrames()) {
                // BoW conversion and insertion in Map
                ProcessNewKeyFrame();

                // Check recent MapPoints
                MapPointCulling();

                // Triangulate new MapPoints
                CreateNewMapPoints(mpCurrentKeyFrame);

                if (!CheckNewKeyFrames()) {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    SearchInNeighbors(mpCurrentKeyFrame);
                }

                mbAbortBA = false;

                if (!CheckNewKeyFrames() && !stopRequested()) {
                    // Local BA
                    if (mpMap->KeyFramesInMap() > 2)
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);

                    // Check redundant local Keyframes
                    KeyFrameCulling();
                }

                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            } else if (Stop()) {
                // Safe area to stop
                while (isStopped() && !CheckFinish()) {
                    usleep(3000);
                }
                if (CheckFinish())
                    break;
            }

            ResetIfRequested();

            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(true);

            if (CheckFinish())
                break;

            usleep(3000);
        }

        SetFinish();
    }

void LocalMapping::RunSequential()
{
    mbFinished = false;

    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(false);

    // Check if there are keyframes in the queue
    if(CheckNewKeyFrames()){

        // BoW conversion and insertion in Map
        ProcessNewKeyFrame();

        // Check recent MapPoints
        MapPointCulling();

        // Triangulate new MapPoints
        CreateNewMapPoints(mpCurrentKeyFrame);

        if(!CheckNewKeyFrames())
        {
            // Find more matches in neighbor keyframes and fuse point duplications
            SearchInNeighbors(mpCurrentKeyFrame);
        }

        // Local BA
        if(mpMap->KeyFramesInMap()>2){
            bool mbAbortBA_sequential = false;
            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA_sequential, mpMap);
        }

        // Check redundant local Keyframes
        KeyFrameCulling();

        mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

    }

    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(true);
}

    void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexNewKFs);
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA = true;
    }


    bool LocalMapping::CheckNewKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKFs);
        return (!mlNewKeyFrames.empty());
    }

    void LocalMapping::ProcessNewKeyFrame() {
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // Compute Bags of Words structures
        mpCurrentKeyFrame->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    } else // this can only happen for new stereo points inserted by the Tracking
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnections();

        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }

    void LocalMapping::MapPointCulling() {
        // Check Recent Added MapPoints
        list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;
        if (mbMonocular)
            nThObs = 2;
        else
            nThObs = 3;
        const int cnThObs = nThObs;

        while (lit != mlpRecentAddedMapPoints.end()) {
            MapPoint *pMP = *lit;
            if (pMP->isBad()) {
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (pMP->GetFoundRatio() < 0.25f) {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs) {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 3)
                lit = mlpRecentAddedMapPoints.erase(lit);
            else
                lit++;
        }
    }

void LocalMapping::CreateNewMapPoints(KeyFrame *keyframe) {
    // Retrieve neighbor keyframes in covisibility graph
    int numberOfNeighbors = 10;
    if (mbMonocular)
        numberOfNeighbors = 20;

    const vector<KeyFrame *> neighbors = keyframe->GetBestCovisibilityKeyFrames(numberOfNeighbors);

    ORBmatcher matcher(0.6, false);

    cv::Mat Rcw1 = keyframe->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = keyframe->GetTranslation();
    cv::Mat Tcw1(3, 4, CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0, 3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = keyframe->GetCameraCenter();

    const float &fx1 = keyframe->fx;
    const float &fy1 = keyframe->fy;
    const float &cx1 = keyframe->cx;
    const float &cy1 = keyframe->cy;
    const float &invfx1 = keyframe->invfx;
    const float &invfy1 = keyframe->invfy;

    const float ratioFactor = 1.5 * keyframe->mfScaleFactor;

    int nnew = 0;

    // Search matches with epipolar restriction and triangulate

    for (size_t i = 0; i < neighbors.size(); i++) {
        if (i > 0 && CheckNewKeyFrames())
            return;

        KeyFrame *neighor = neighbors[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = neighor->GetCameraCenter();
        cv::Mat vBaseline = Ow2 - Ow1;
        const float baseline = cv::norm(vBaseline);

        if (!mbMonocular) {
            if (baseline < neighor->mb)
                continue;
        } else {
            const float medianDepthKF2 = neighor->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (ratioBaselineDepth < 0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(keyframe, neighor);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t, size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(keyframe, neighor, F12, vMatchedIndices, false);

        cv::Mat Rcw2 = neighor->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = neighor->GetTranslation();
        cv::Mat Tcw2(3, 4, CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0, 3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = neighor->fx;
        const float &fy2 = neighor->fy;
        const float &cx2 = neighor->cx;
        const float &cy2 = neighor->cy;
        const float &invfx2 = neighor->invfx;
        const float &invfy2 = neighor->invfy;

        const int nmatches = vMatchedIndices.size();

        // Triangulate each match
        for (int ikp = 0; ikp < nmatches; ikp++) {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = keyframe->mvKeysUn[idx1];
            const float kp1_ur = keyframe->mvuRight[idx1];
            bool bStereo1 = kp1_ur >= 0;

            const cv::KeyPoint &kp2 = neighor->mvKeysUn[idx2];
            const float kp2_ur = neighor->mvuRight[idx2];
            bool bStereo2 = kp2_ur >= 0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

            cv::Mat ray1 = Rwc1 * xn1;
            cv::Mat ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (bStereo1)
                cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
            else if (bStereo2)
                cosParallaxStereo2 = cos(2 * atan2(neighor->mb / 2, neighor->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

            cv::Mat x3D;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
            (bStereo1 || bStereo2 || cosParallaxRays < 0.9998)) {
                // Linear Triangulation Method
                cv::Mat A(4, 4, CV_32F);
                A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
                A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
                A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
                A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

                cv::Mat w, u, vt;
                cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if (x3D.at<float>(3) == 0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
            } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
                x3D = keyframe->UnprojectStereo(idx1);
                } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
                    x3D = neighor->UnprojectStereo(idx2);
                    } else
                        continue; //No stereo and very low parallax
            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
            if (z1 <= 0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
            if (z2 <= 0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = keyframe->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
            const float invz1 = 1.0 / z1;

            if (!bStereo1) {
                float u1 = fx1 * x1 * invz1 + cx1;
                float v1 = fy1 * y1 * invz1 + cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                    continue;
            } else {
                float u1 = fx1 * x1 * invz1 + cx1;
                float u1_r = u1 - keyframe->mbf * invz1;
                float v1 = fy1 * y1 * invz1 + cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = neighor->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
            const float invz2 = 1.0 / z2;
            if (!bStereo2) {
                float u2 = fx2 * x2 * invz2 + cx2;
                float v2 = fy2 * y2 * invz2 + cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                    continue;
            } else {
                float u2 = fx2 * x2 * invz2 + cx2;
                float u2_r = u2 - keyframe->mbf * invz2;
                float v2 = fy2 * y2 * invz2 + cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D - Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D - Ow2;
            float dist2 = cv::norm(normal2);

            if (dist1 == 0 || dist2 == 0)
                continue;

            const float ratioDist = dist2 / dist1;
            const float ratioOctave = keyframe->mvScaleFactors[kp1.octave] / neighor->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
            continue;*/
            if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint *pMP = keyframe->CreateMapPoint(x3D, KeypointIndex(idx1));

            pMP->AddObservation(neighor, KeypointIndex(idx2));
            neighor->AddMapPoint(pMP, KeypointIndex(idx2));

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors(Keyframe keyframe)
{
    // Retrieve neighbor keyframes
    int numberOfNeighbors = 10;
    if(mbMonocular)
        numberOfNeighbors = 20;

    const vector<KeyFrame*> neighbors = keyframe->GetBestCovisibilityKeyFrames(numberOfNeighbors);

    vector<KeyFrame*> targetKeyframes;
    set<KeyFrame*> targetKeyframes_;

    for(auto& neighbor: neighbors){

        if(neighbor->isBad() || targetKeyframes_.count(neighbor))
            continue;

        targetKeyframes.push_back(neighbor);
        targetKeyframes_.insert(neighbor);

        // Extend to some second neighbors
        const vector<KeyFrame*> secondNeighbors = neighbor->GetBestCovisibilityKeyFrames(5);
        for(auto& secondNeighbor: secondNeighbors)
        {
            if(secondNeighbor->isBad() || targetKeyframes_.count(secondNeighbor) || secondNeighbor->mnId == keyframe->mnId)
                continue;

            targetKeyframes.push_back(secondNeighbor);
            //targetKeyframes_.insert(secondNeighbor);
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> mapPointMatches = keyframe->GetMapPointMatches();
    for(auto& targetKeyframe: targetKeyframes)
        matcher.Fuse(targetKeyframe,mapPointMatches,3.0f);

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> fuseCandidates;
    set<MapPoint*> fuseCandidates_;
    fuseCandidates.reserve(targetKeyframes.size() * mapPointMatches.size());

    for(auto& targetKeyframe: targetKeyframes)
    {
        vector<MapPoint*> mapPoints = targetKeyframe->GetMapPointMatches();

        for(auto& mapPoint: mapPoints)
        {
            if((!mapPoint) || mapPoint->isBad() || fuseCandidates_.count(mapPoint))
                continue;

            fuseCandidates.push_back(mapPoint);
            fuseCandidates_.insert(mapPoint);
        }
    }

    matcher.Fuse(keyframe,fuseCandidates,3.0);
    // Update points
    mapPointMatches = keyframe->GetMapPointMatches();
    for(auto& mapPoint: mapPointMatches)
        if(mapPoint)
            if(!mapPoint->isBad())
                mapPoint->ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();

    // Update connections in covisibility graph
    keyframe->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
#ifdef COMPILED_SEQUENTIAL
    Stop();
#endif
}

bool LocalMapping::Stop()
{
#ifndef COMPILED_SEQUENTIAL
    unique_lock<mutex> lock(mMutexStop);
#endif
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<Keyframe> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(auto& keyframe: vpLocalKeyFrames)
    {
        if(keyframe->mnId==0)
            continue;

        const vector<MapPt> vpMapPoints = keyframe->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        int iMapPt{0};
        for(auto& mapPt: vpMapPoints)
        {
            if(mapPt)
            {
                if(!mapPt->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(keyframe->mvDepth[iMapPt] > keyframe->mThDepth || keyframe->mvDepth[iMapPt] < 0)
                            continue;
                    }

                    nMPs++;
                    if(mapPt->Observations() > thObs)
                    {
                        const int &scaleLevel = keyframe->mvKeysUn[iMapPt].octave;
                        map<KeyframeId , Observation> observations = mapPt->GetObservations();
                        int numberOfObservations = 0;
                        for(auto& obs: observations)
                        {
                            KeyFrame* projKeyframe = obs.second.projKeyframe;
                            if(projKeyframe == keyframe)
                                continue;

                            const int &scaleLeveli = projKeyframe->mvKeysUn[obs.second.projIndex].octave;

                            if(scaleLeveli <= scaleLevel + 1)
                            {
                                numberOfObservations++;
                                if(numberOfObservations >= thObs)
                                    break;
                            }
                        }
                        if(numberOfObservations >= thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
            ++iMapPt;
        }  

        if(nRedundantObservations > 0.9*nMPs)
            keyframe->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }
#ifndef COMPILED_SEQUENTIAL
    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
#else
   ResetIfRequested();
#endif
}

void LocalMapping::ResetIfRequested()
{
#ifndef COMPILED_SEQUENTIAL
    unique_lock<mutex> lock(mMutexReset);
#endif
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested = false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
