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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

Observation::Observation(KeyFrame* projKeyframe, const KeypointIndex& projIndex,
                         KeyFrame* refKeyframe , const KeypointIndex& refIndex):
                         projKeyframe(projKeyframe), projIndex(projIndex),
                        refKeyframe(refKeyframe), refIndex(refIndex){

        projOctave = projKeyframe->mvKeysUn[projIndex].octave;
        refOctave = refKeyframe->mvKeysUn[refIndex].octave;
}

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;

    currentReferenceKeyframe = mpRefKF;
    currentReferenceKeypointIndex = -1;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;

    currentReferenceKeyframe = nullptr;
    currentReferenceKeypointIndex = -1;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* projKeyframe, const KeypointIndex& projIndex)
{
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(projKeyframe->mnId))
            return;
        mObservations[projKeyframe->mnId] = Observation(projKeyframe, projIndex,
                                                        currentReferenceKeyframe,
                                                        currentReferenceKeypointIndex);
        increasePointObservability(projKeyframe,projIndex);
    }

    ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();

}

void MapPoint::EraseObservation(KeyFrame* projKeyframe)
{
    bool removePoint{false};
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(projKeyframe->mnId))
        {
            KeypointIndex projIndex = mObservations[projKeyframe->mnId].projIndex;
            decreasePointObservability(projKeyframe,projIndex);
            mObservations.erase(projKeyframe->mnId);
            if(mpRefKF == projKeyframe)
                mpRefKF = mObservations.begin()->second.projKeyframe;

            // If only 2 observations or less, discard point
            removePoint = (GetNumberOfObservations() <= 2);
        }
    }

    if(removePoint)
        SetBadFlag();
    else
        ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();
}

std::map<KeyframeId, Observation> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

int MapPoint::GetNumberOfObservations()
{
    return int(mObservations.size());
}

void MapPoint::increasePointObservability(KeyFrame* projKeyframe, const KeypointIndex& projIndex){
    if(projKeyframe->mvuRight[projIndex] >= 0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::decreasePointObservability(KeyFrame* projKeyframe, const KeypointIndex& projIndex){
    if(projKeyframe->mvuRight[projIndex] >= 0)
        nObs-=2;
    else
        nObs--;
}

void MapPoint::SetCurrentRefKeyframeIndex(const KeypointIndex& refKeyframeIndex){
    currentReferenceKeypointIndex = refKeyframeIndex;
}

KeyFrame* MapPoint::GetCurrentRefKeyframe(){
    return currentReferenceKeyframe;
}

void MapPoint::SetBadFlag()
{
    map<KeyframeId , Observation> observations;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        observations = mObservations;
        mObservations.clear();
    }
    for(auto& obs: observations){
        Keyframe keyframe = obs.second.projKeyframe;
        keyframe->EraseMapPointMatch(obs.second.projIndex);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPt mapPt)
{
    if(mapPt->mnId == this->mnId)
        return;

    int nvisible, nfound;
    map<KeyframeId , Observation> observations;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        observations = mObservations;
        mObservations.clear();
        mbBad = true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = mapPt;
    }

    for(auto& obs: observations)
    {
        // Replace measurement in keyframe
        Keyframe keyframe = obs.second.projKeyframe;

        if(!mapPt->IsInKeyFrame(keyframe))
        {
            keyframe->ReplaceMapPointMatch(obs.second.projIndex, mapPt);
            mapPt->AddObservation(keyframe,obs.second.projIndex);
        }
        else
        {
            keyframe->EraseMapPointMatch(obs.second.projIndex);
        }
    }

    mapPt->IncreaseFound(nfound);
    mapPt->IncreaseVisible(nvisible);
    mapPt->ComputeDistinctiveDescriptors()->UpdateNormalAndDepth();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

MapPt MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> descriptors;
    vector<KeypointIndex> projIndexes{};
    vector<Keyframe> projectionKeyframes{};

    map<KeyframeId, Observation> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return this;
    }

    observations = GetObservations();

    if(observations.empty())
        return this;

    descriptors.reserve(observations.size());
    for(auto& obs: observations)
    {
        Keyframe projKeyframe = obs.second.projKeyframe;
        if(!projKeyframe->isBad()){
            descriptors.push_back(projKeyframe->mDescriptors.row(obs.second.projIndex));
            projIndexes.push_back(obs.second.projIndex);
            projectionKeyframes.push_back(projKeyframe);
        }
    }

    if(descriptors.empty())
        return this;

    // Compute distances between descriptors
    const size_t N = descriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(descriptors[i],descriptors[j]);
            Distances[i][j] = float(distij);
            Distances[j][i] = float(distij);
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = descriptors[BestIdx].clone();
        currentReferenceKeypointIndex = projIndexes[BestIdx];
        currentReferenceKeyframe = projectionKeyframes[BestIdx];
    }

    return this;
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame* keyframe)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(keyframe->mnId))
        return mObservations[keyframe->mnId].projIndex;
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame* keyframe)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(keyframe->mnId));
}

void MapPoint::UpdateNormalAndDepth()
{

    map<KeyframeId , Observation> observations = GetObservations();
    if(observations.empty())
        return;

    Keyframe refKeyframe;
    cv::Mat XYZ;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;

        refKeyframe = mpRefKF;
        XYZ = mWorldPos.clone();
    }

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n = 0;

    for(const auto& obs: observations)
    {
        KeyFrame* projKeyframe = obs.second.projKeyframe;
        cv::Mat cameraCenter = projKeyframe->GetCameraCenter();
        cv::Mat normal_i = XYZ - cameraCenter;
        normal = normal + normal_i/cv::norm(normal_i);
        n++;
    }

    cv::Mat PC = XYZ - refKeyframe->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = refKeyframe->mvKeysUn[observations[refKeyframe->mnId].projIndex].octave;
    const float levelScaleFactor =  refKeyframe->mvScaleFactors[level];
    const int nLevels = refKeyframe->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/refKeyframe->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
