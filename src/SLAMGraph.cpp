//
// Created by fontan on 15/06/23.
//

#include "SLAMGraph.h"

SLAM_GRAPH::FrameNODE::FrameNODE(const FrameId& frameId, const Seconds& timestamp,
                                 const mat4& Tr_, const shared_ptr<KeyframeNODE>& refKeyframe):
                                    frameId(frameId),timestamp(timestamp){
    Tr.insert(make_pair(refKeyframe, vector<mat4>{Tr_}));
}
void SLAM_GRAPH::SLAMGraph::initialize(const FrameId& frameId_1, const Seconds& timestamp_1, const mat4& Twc_1,
                                       const FrameId& frameId_2, const Seconds& timestamp_2, const mat4& Twc_2){
    unique_lock<mutex> lock(mutexSlamGraph);

    KeyframeNode keyframe_1 = make_shared<KeyframeNODE>(KeyframeNODE(frameId_1,timestamp_1, Twc_1));
    keyframes.insert(make_pair(frameId_1, keyframe_1));

    KeyframeNode keyframe_2 = make_shared<KeyframeNODE>(KeyframeNODE(frameId_2,timestamp_2, Twc_2));
    keyframes.insert(make_pair(frameId_2, keyframe_2));

    linkKeyframes(keyframe_1, keyframe_2);
}

void SLAM_GRAPH::SLAMGraph::addKeyframe(const FrameId& frameId, const Seconds& timestamp, const mat4& Twc, const FrameId& refKeyframeId){
    unique_lock<mutex> lock(mutexSlamGraph);

    if(keyframes.count(frameId)) { // Check if keyframe already exists
        cout << "SLAM_GRAPH::SLAMGraph::addKeyframe: FAILED" << endl;
        cout << "     Keyframe " << frameId << " already exists."<<endl;
        return;
    }
    if(!keyframes.count(refKeyframeId)) { // Check if reference keyframe exists
        cout << "SLAM_GRAPH::SLAMGraph::addKeyframe: FAILED" << endl;
        cout << "     Reference Keyframe " << refKeyframeId << " doesn't exist."<<endl;
        return;
    }

    KeyframeNode newKeyframe= make_shared<KeyframeNODE>(KeyframeNODE(frameId,timestamp, Twc));
    keyframes.insert(make_pair(frameId, newKeyframe));

    KeyframeNode refKeyframe = keyframes.find(refKeyframeId)->second;
    linkKeyframes(newKeyframe, refKeyframe);
}

void SLAM_GRAPH::SLAMGraph::removeKeyframe(const FrameId &keyframeToRemoveId){
    unique_lock<mutex> lock(mutexSlamGraph);
    auto keyframeToRemove_ = keyframes.find(keyframeToRemoveId);
    if(keyframeToRemove_ == keyframes.end()) { // Check if keyframe exists
        cout << "SLAM_GRAPH::SLAMGraph::removeKeyframe: FAILED" << endl;
        cout << "     Keyframe " << keyframeToRemoveId << " doesn't exist."<<endl;
        return;
    }

    auto keyframeToRemove = keyframeToRemove_->second;
    auto auxKeyframe = keyframeToRemove->getAuxiliaryCovisibleKeyframe();
    for(auto& covKeyframe: keyframeToRemove->getCovisibleKeyframes()){
        covKeyframe.second->removeCovisibleKeyframe(keyframeToRemoveId, auxKeyframe);
    }
    keyframes.erase(keyframeToRemoveId);
}

void SLAM_GRAPH::SLAMGraph::updateKeyframePose(const FrameId& frameId,const mat4& Twc){
    unique_lock<mutex> lock(mutexSlamGraph);
    if(!keyframes.count(frameId)){
        return;
    }
    KeyframeNode keyframe = keyframes.find(frameId)->second;
    keyframe->updatePose(Twc);
}

void SLAM_GRAPH::SLAMGraph::linkKeyframes(KeyframeNode& keyframe1, KeyframeNode& keyframe2){
    keyframe1->addCovisibleKeyframe(keyframe2);
    keyframe2->addCovisibleKeyframe(keyframe1);
}

/*void SLAM_GRAPH::KeyframeNODE::addFrame(const FrameId& frameId_){
    if(frames.count(frameId_)) // Check if frame already exists
        return;
    if(frameId == frameId_) // Check if frame already exists
        return;

    frames.insert(frameId);
}*/

/*void SLAM_GRAPH::SLAMGraph::addFrame(const FrameId &frameId, const Seconds& timestamp, const mat4& Tr_, const FrameId& refKeyframeId){
    if(frames.count(frameId)) { // Check if frame already exists
        if(!frames.find(frameId)->second.isKeyframe()){
            cout << "SLAM_GRAPH::SLAMGraph::addFrame: FAILED" << endl;
            cout << "     Frame " << frameId << " already exists."<<endl;
        }
        return;
    }

    auto refKeyframe = keyframes.find(refKeyframeId);
    if(refKeyframe == keyframes.end()){ // Check if keyframe exists
        cout << "SLAM_GRAPH::SLAMGraph::addFrame: FAILED" << endl;
        cout << "     Keyframe " << refKeyframeId << " doesn't exist."<<endl;
        return;
    }

    FrameNODE frameNODE(frameId,timestamp,Tr_, make_shared<KeyframeNODE>(refKeyframe->second));
    frames.insert(make_pair(frameId, frameNODE));
    refKeyframe->second.addFrame(frameId);
}*/

bool SLAM_GRAPH::FrameNODE::isKeyframe() const{
    return nodeIsKeyframe;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keyframe Node Methods
SLAM_GRAPH::KeyframeNODE::KeyframeNODE(const FrameId& frameId_, const Seconds& timestamp_, const mat4& Twc_):
        frameId(frameId_),timestamp(timestamp_),Twc(Twc_){}

void SLAM_GRAPH::KeyframeNODE::addCovisibleKeyframe(const KeyframeNode& covKeyframe){
    if(covisibleKeyframes.count(covKeyframe->frameId))
        return;
    covisibleKeyframes.insert(make_pair(covKeyframe->frameId, covKeyframe));
}

void SLAM_GRAPH::KeyframeNODE::removeCovisibleKeyframe(const FrameId& covKeyframeId, const KeyframeNode& auxKeyframe){
    if(!covisibleKeyframes.count(covKeyframeId))
        return;
    covisibleKeyframes.erase(covKeyframeId);
    if(covisibleKeyframes.empty())
        covisibleKeyframes.insert(make_pair(auxKeyframe->frameId, auxKeyframe));
}

SLAM_GRAPH::KeyframeNode SLAM_GRAPH::KeyframeNODE::getAuxiliaryCovisibleKeyframe(){
    return covisibleKeyframes.begin()->second;
}

void SLAM_GRAPH::KeyframeNODE::updatePose(const mat4& Twc_){
    Twc = Twc_;
}