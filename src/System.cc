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



#include "System.h"
#include "Converter.h"
#include "Optimizer.h"
#include "DistributionFitter.h"

#include <thread>
#include <iomanip>
#include <iostream>
#include <fstream>

#ifdef COMPILED_WITH_PANGOLIN
#include<pangolin/pangolin.h>
#endif

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const int expId,
               const bool bUseViewer,
               const string resultsPath):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false), expId(expId), resultsPath(resultsPath)
{

    RandomIntegerGenerator::seedRandomGenerator();

    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    cout << "Experiment Index is : " << expId << endl;

    //Check settings file
    cv::FileStorage settingsFile(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!settingsFile.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Load SLAM parameters

    // Load Resolution parameters
    int cropBottom_ = settingsFile["Resolution.cropBottom"];
    if(cropBottom_ != 0)
        cropBottom = cropBottom_;

    cout << "Resolution Parameters"<< endl;
    cout << "- Crop Bottom: "<< cropBottom <<endl;

    // Load Optimizer parameters
    OptimizerParameters::PoseOptimizationParameters poseOptimizationParameters(
            settingsFile["Optimizer.poseOptimization.nInitialCorrespondences"],
            settingsFile["Optimizer.poseOptimization.optimizerIts"],
            settingsFile["Optimizer.poseOptimization.its"],
            settingsFile["Optimizer.poseOptimization.minimumNumberOfEdges"]);

    OptimizerParameters::LocalBundleAdjustmentParameters localBundleAdjustmentParameters(
            settingsFile["Optimizer.localBundleAdjustment.optimizerItsCoarse"],
            settingsFile["Optimizer.localBundleAdjustment.optimizerItsFine"]);

    OptimizerParameters::OptimizeEssentialGraph optimizeEssentialGraph(
            settingsFile["Optimizer.optimizeEssentialGraph.solverLambdaInit"],
            settingsFile["Optimizer.optimizeEssentialGraph.minFeat"],
            settingsFile["Optimizer.optimizeEssentialGraph.optimizerIts"]);

    OptimizerParameters::OptimizeSim3 optimizeSim3(
            settingsFile["Optimizer.optimizeSim3.optimizerIts"],
            settingsFile["Optimizer.optimizeSim3.nMoreIterationsHigh"],
            settingsFile["Optimizer.optimizeSim3.nMoreIterationsLow"],
            settingsFile["Optimizer.optimizeSim3.nBad"],
            settingsFile["Optimizer.optimizeSim3.th2"]);

    OptimizerParameters::GlobalRobustBundleAdjustment globalRobustBundleAdjustment(
            settingsFile["Optimizer.globalRobustBundleAdjustment.optimizerItsCoarse"],
            settingsFile["Optimizer.globalRobustBundleAdjustment.optimizerItsFine"]);

    Optimizer::parameters.setParameters(settingsFile["Optimizer.chi2_2dof"], settingsFile["Optimizer.chi2_3dof"],
                                        settingsFile["Optimizer.inlierProbability"],
                                        poseOptimizationParameters,
                                        localBundleAdjustmentParameters,
                                        optimizeEssentialGraph,
                                        optimizeSim3,
                                        globalRobustBundleAdjustment);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Create SLAM Graph
    slamGraph = make_shared<SLAM_GRAPH::SLAMGraph>(SLAM_GRAPH::SLAMGraph::LOW);
    KeyFrame::slamGraph = slamGraph;

    // Distribution Fitter
    DIST_FITTER::DistributionFitterParameters::LogNormal logNormal(
            settingsFile["DistributionFitter.logNormal.maxNumberIterations"],
            settingsFile["DistributionFitter.logNormal.stepSize"],
            settingsFile["DistributionFitter.logNormal.tolerance"]);
    DIST_FITTER::DistributionFitterParameters::Burr burr(
            settingsFile["DistributionFitter.burr.maxNumberIterations"],
            settingsFile["DistributionFitter.burr.stepSize"],
            settingsFile["DistributionFitter.burr.tolerance"]);

    DIST_FITTER::DistributionFitter::verbosity = DIST_FITTER::DistributionFitter::VerbosityLevel::MEDIUM;
    DIST_FITTER::DistributionFitter::params.SetParameters(logNormal,burr);

#ifdef COMPILED_ABLATION
    vector<double> probabilities{0.5,0.6,
                                 0.7,0.725,0.75,0.775,
                                 0.8,0.8125,0.825,0.8375,0.85,0.8625,0.875,0.8875,
                                 0.9,0.9125,0.925,0.9375,0.95,0.9625,0.975,0.9875,
                                 0.99,0.995,0.999};

    vector<double> chi2{0.0625,0.125,0.25,
                        0.5,1,2,
                        3,4,4.5,
                        5,5.5,5.75,
                        5.991,
                        6.25,6.5,7,
                        7.5,8,9,
                        16,32,64,
                        128,256,512};

    //Optimizer::parameters.UpdateInlierProbability(probabilities[expId]);
    //Optimizer::parameters.UpdateInlierThresholds(chi2[expId],chi2[expId]);
#endif

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, slamGraph,mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap,slamGraph, mSensor==MONOCULAR);
#ifndef COMPILED_SEQUENTIAL
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
#endif

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
#ifndef COMPILED_SEQUENTIAL
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
#endif

#ifdef COMPILED_WITH_PANGOLIN
    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }
#endif

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Print parameters
    cout << Optimizer::parameters;
    cout << DIST_FITTER::DistributionFitter::params;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::GlobalBundleAdjustment(){
    cout << "GlobalBundleAdjustment ... "<< endl;
    Optimizer::GlobalBundleAdjustment(mpMap, 100, nullptr, 0,true);
}

void System::GlobalRobustBundleAdjustment(){
    cout << "GlobalRobustBundleAdjustment ... "<< endl;
    Optimizer::GlobalRobustBundleAdjustment(mpMap);
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

///////////////////////////////// Ablation Functions
void UpdateProbability(const double& probability){
    Optimizer::parameters.UpdateInlierProbability(probability);
}

void UpdateChi2(const double& chi2){
    Optimizer::parameters.UpdateInlierThresholds(chi2,chi2);
}

///////////////////////////////// Ablation Functions

    void System::Shutdown()
{

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

#ifndef COMPILED_SEQUENTIAL
    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
#endif

#ifdef COMPILED_WITH_PANGOLIN
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
#endif

#ifdef COMPILED_ABLATION_GBA

    cout << "\n[GBA Ablation] exp id " << expId << " starts ... "<< endl;

    string resultsPath_expId = resultsPath + "/" + ORB_SLAM2::paddingZeros(to_string(expId));

    // Save trajectory before Global Bundle Adjustment
    SaveFrameTrajectoryTUM(resultsPath_expId + "_FrameTrajectoryBeforeBA.txt");
    SaveKeyFrameTrajectoryTUM(resultsPath_expId + "_KeyFrameTrajectoryBeforeBA.txt");

    // Global Bundle Adjustment
    GlobalRobustBundleAdjustment();

    // Save trajectory after Global Bundle Adjustment
    SaveFrameTrajectoryTUM(resultsPath_expId + "_FrameTrajectoryAfterBA.txt");
    SaveKeyFrameTrajectoryTUM(resultsPath_expId + "_KeyFrameTrajectoryAfterBA.txt");

    // Save a copy of the Map (keyframes and mapPoints)
    for(auto& mapPoint: mpMap->GetAllMapPoints())
        slamGraph->addMapPoint(mapPoint->mnId,mapPoint->GetXYZ());
    slamGraph->saveMap();

    // Add noise to the saved copy
    slamGraph->addNoiseToSavedMap(0.1);

    // Define Ablation Variable
    string ablationVariableName1{"probability"};
    vector<double> ablationVariable1{0.5,0.6,0.7,0.8,0.9,0.95};
    Optimizer::parameters.estimateThreshold = true;
    GBA_ablation(ablationVariable1,ablationVariableName1,UpdateProbability);

    // Define Ablation Variable
    string ablationVariableName2{"chi2"};
    vector<double> ablationVariable2{2.0,3.0,5.9,6.0};
    Optimizer::parameters.estimateThreshold = false;
    GBA_ablation(ablationVariable2,ablationVariableName2, UpdateChi2);

#endif
}

void System::GBA_ablation(const vector<double>& ablationVariable, const string& ablationVariableName,void (*operation)(const double&)){

    // Create log file
    const std::string filename = resultsPath + "/" + ablationVariableName +  "_GBA_Ablation_log.txt";
    std::ofstream file(filename, std::ios::out | std::ios::app);

    // Ablation results path
    string resultsPath_expId = resultsPath + "/" + ORB_SLAM2::paddingZeros(to_string(expId));

    // Ablation loop
    for(int ablationId{0}; ablationId < ablationVariable.size(); ablationId++){
        // Reset Map (keyframes and mapPoints) from copy
        slamGraph->resetMapFromCopy();
        for(auto& keyframe: mpMap->GetAllKeyFrames())
            keyframe->SetPose(Converter::toCvMat(slamGraph->getTcw(keyframe->mnFrameId)));
        for(auto& mapPoint: mpMap->GetAllMapPoints())
            mapPoint->SetWorldPos(Converter::toCvMat(slamGraph->getXYZ(mapPoint->mnId)));

        // Update ablation variable
        cout << "Ablation "<< ablationId << " with variable value = " << ablationVariable[ablationId] << endl;
        operation(ablationVariable[ablationId]);

        // Global Bundle Adjustment
        GlobalRobustBundleAdjustment();

        // Save iteration results
        string resultsPath_ = resultsPath_expId + "_" + ORB_SLAM2::paddingZeros(to_string(ablationId));
        SaveFrameTrajectoryTUM(resultsPath_ + "_" + ablationVariableName + "_AblationFrame.txt");
        SaveKeyFrameTrajectoryTUM(resultsPath_ + "_" + ablationVariableName + "_AblationKeyFrame.txt");
        file << resultsPath_ + "_AblationFrame.txt " << expId << " " << ablationId << " " << ablationVariable[ablationId]  << std::endl;
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving frame trajectory to " << filename << " ..." << endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(const auto& frameId: slamGraph->getIds()){
        mat4 Twc = slamGraph->getTwc(frameId);
        mat3 Rwc = Twc.block<3,3>(0,0);
        vec3 twc = Twc.block<3,1>(0,3);
        quat q(Rwc);
        f << setprecision(6) << slamGraph->getTimestamp(frameId) << setprecision(7)
        << " " << twc(0) << " " << twc(1) << " " << twc(2)
        << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    auto keyframes = slamGraph->getKeyframes();
    for(const auto& keyframe: *keyframes){
        mat4 Twc = keyframe.second->getTwc();
        mat3 Rwc = Twc.block<3,3>(0,0);
        vec3 twc = Twc.block<3,1>(0,3);
        quat q(Rwc);
        f << setprecision(6) << keyframe.second->getTimestamp() << setprecision(7)
          << " " << twc(0) << " " << twc(1) << " " << twc(2)
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::readImage(cv::Mat& im, const string& imagePath){
    im = cv::imread(imagePath,cv::IMREAD_UNCHANGED);
    if(im.empty())
    {
        cerr << endl << "Failed to load image at: " << imagePath << endl;
        return;
    }

#ifdef COMPILED_FITRESOLUTION
    static bool firstImage{true};

    // Undistort Image
    static cv::Mat map1GrayImage,map2GrayImage;
    static float resolutionFactor_x, resolutionFactor_y;
    if(firstImage)
        mpTracker->UndistortCalibration(im.cols,im.rows, map1GrayImage,map2GrayImage);
    cv::Mat imageTemporal = im.clone();
    cv::remap(imageTemporal, im, map1GrayImage, map2GrayImage, cv::INTER_LINEAR);

    if(firstImage){
        float numPixels = 640.0f*480.0f;
        float ratio = float(im.cols)/float(im.rows);
        float h = sqrt(numPixels/ratio);
        float w = h * ratio;
        resolutionFactor_x = float(w)/float(im.cols);
        resolutionFactor_y = float(h)/float(im.rows);

        mpTracker->ResizeCalibration(resolutionFactor_x, resolutionFactor_y);
        firstImage = false;
    }
    cv::resize(im.clone(),im, cv::Size(), resolutionFactor_x, resolutionFactor_y, cv::INTER_LINEAR);
#endif

    // Define the region of interest (ROI) to crop
    cv::Rect roi(0, 0, im.cols, im.rows - cropBottom); // (x, y, width, height)

    // Crop the image based on the defined ROI
    im = im(roi);

}

void System::SaveStatisticsToFiles(const string& pathToFiles){
    cout << "Saving Statistics Files: "<< pathToFiles << endl;
#ifdef COMPILED_DEBUG
#ifdef COMPILED_ABLATION_GBA
    cout << "    "<< "residuals_u.txt" << endl;
    saveVectorToFile(Optimizer::residuals_u,pathToFiles + "residuals_u.txt");
    cout << "    "<< "residuals_v.txt" << endl;
    saveVectorToFile(Optimizer::residuals_v,pathToFiles + "residuals_v.txt");
#endif

    cout << "    "<< "inlierThreshold.txt" << endl;
    saveVectorToFile(Optimizer::inlierThreshold,pathToFiles + "inlierThreshold.txt");

    cout << "    "<< "outlierPercentage.txt" << endl;
    saveVectorToFile(Optimizer::outlierPercentage,pathToFiles + "outlierPercentage.txt");

    cout << "    "<< "ablationParameters.txt" << endl;
    vector<double> ablationParameters{Optimizer::parameters.inlierProbability,Optimizer::parameters.pExp, Optimizer::parameters.th2_2dof};
    saveVectorToFile(ablationParameters,pathToFiles + "ablationParameters.txt");

    auto mapPoints = mpMap->GetAllMapPoints();
    int numberOfObservations = 0;
    int numberOfActiveObservations = 0;
    for(auto& mapPt: mapPoints){
        numberOfObservations += mapPt->GetNumberOfObservations();
        numberOfActiveObservations += mapPt->GetPointObservability();
    }
    cout <<  "Number Of Observations: " << numberOfObservations<< endl;
    cout <<  "Number Of Active Observations: " << numberOfActiveObservations<< endl;
    cout <<  "Active Observations Percentaje: " << 100.0*double(numberOfActiveObservations)/double(numberOfObservations) << " %"<< endl;
#endif
}

void System::saveMap(){
    for( const auto& mapPt: mpMap->GetAllMapPoints()){
        slamGraph->addMapPoint(mapPt->mnId,mapPt->GetXYZ());
    }
    slamGraph->saveMap();
}

void System::loadMap(){
    slamGraph->resetMapFromCopy();
    for(auto& keyframe:  mpMap->GetAllKeyFrames())
        keyframe->SetPose(Converter::toCvMat(slamGraph->getTcw(keyframe->mnFrameId)));

    for( const auto& mapPt:  mpMap->GetAllMapPoints()) {
        mapPt->SetWorldPos(Converter::toCvMat(slamGraph->getXYZ(mapPt->mnId)));
        mapPt->UpdateNormalAndDepth();
    }
}


} //namespace ORB_SLAM

