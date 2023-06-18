//
// Created by fontan on 15/06/23.
//

#include "SLAMGraph.h"

#include <utility>

void SLAM_GRAPH::SLAMGraph::initialize(const FrameId& id_1, const Seconds& timestamp_1, const mat4& Twc_1,
                                       const FrameId& id_2, const Seconds& timestamp_2, const mat4& Twc_2){
    KeyframeNode keyframe_1 = make_shared<KeyframeNODE>(KeyframeNODE(id_1,timestamp_1, Twc_1));
    keyframes.insert(make_pair(id_1, keyframe_1));

    KeyframeNode keyframe_2 = make_shared<KeyframeNODE>(KeyframeNODE(id_2,timestamp_2, Twc_2));
    keyframes.insert(make_pair(id_2, keyframe_2));

}

void SLAM_GRAPH::SLAMGraph::addKeyframe(const FrameId& keyframeId, const Seconds& timestamp, const mat4& Twc){
    if(keyframes.count(keyframeId)) { // Check if keyframe already exists
        cout << "[SLAMGraph::addKeyframe]: FAILED" << endl;
        cout << "     Keyframe " << keyframeId << " already exists."<<endl;
        return;
    }

    KeyframeNode newKeyframe = make_shared<KeyframeNODE>(KeyframeNODE(keyframeId,timestamp, Twc));
    keyframes.insert(make_pair(keyframeId, newKeyframe));
    ids.insert(keyframeId);
}

void SLAM_GRAPH::SLAMGraph::addFrame(const FrameId &frameId, const Seconds& timestamp, const mat4& Tcw, const FrameId& refKeyframeId){
    if(frames.count(frameId)) { // Check if frame already exists
        cout << "[SLAMGraph::addFrame]: FAILED" << endl;
        cout << "     Frame " << frameId << " already exists."<<endl;
        return;
    }

    if(keyframes.count(frameId)) { // Check if frame already exists
        return;
    }

    if(!keyframes.count(refKeyframeId)){ // Check if keyframe exists
        cout << "[SLAMGraph::addFrame]: FAILED" << endl;
        cout << "     Reference Keyframe " << refKeyframeId << " doesn't exist."<<endl;
        return;
    }

    KeyframeNode refKeyframe = keyframes.find(refKeyframeId)->second;
    mat4 Tr = Tcw * refKeyframe->getTwc();

    FrameNode newFrame = make_shared<FrameNODE>(FrameNODE(frameId,timestamp,Tr,refKeyframe));
    frames.insert(make_pair(frameId, newFrame));
    refKeyframe->addFrame(newFrame);
    ids.insert(frameId);
}

void SLAM_GRAPH::SLAMGraph::removeKeyframe(const FrameId &keyframeToRemoveId){
    if(!keyframes.count(keyframeToRemoveId)) { // Check if keyframe exists
        cout << "[SLAMGraph::removeKeyframe]: FAILED" << endl;
        cout << "     Keyframe " << keyframeToRemoveId << " doesn't exist."<<endl;
        return;
    }

    KeyframeNode keyframeToRemove = keyframes.find(keyframeToRemoveId)->second;
    auto associatedFrames = keyframeToRemove->getFrames();
    mat4 Tcw_keyframeToRemove = keyframeToRemove->getTcw();
    Seconds timestamp_keyframeToRemove = keyframeToRemove->getTimestamp();

    keyframes.erase(keyframeToRemoveId);

    // Update associated frames
    for(auto& frame: associatedFrames){
        if(frame.second->amountOfRelativePoses() < 2){
            KeyframeNode newRefKeyframe = getNewReferenceKeyframe(frame.first);
            mat4 Twc_newRefKeyframe =  newRefKeyframe->getTwc();
            mat4 Tr_new = Tcw_keyframeToRemove * Twc_newRefKeyframe;
            mat4 Tr_old = frame.second->getTr(keyframeToRemove);
            mat4 Tr = Tr_old * Tr_new;
            frame.second->addTr(newRefKeyframe, Tr);
            newRefKeyframe->addFrame(frame.second);
        }
        frame.second->removeTr(keyframeToRemove);
    }

    // Create new frame from removed keyframe
    KeyframeNode newRefKeyframe = getNewReferenceKeyframe(keyframeToRemoveId);
    mat4 Twc_newRefKeyframe =  newRefKeyframe->getTwc();
    mat4 Tr_new = Tcw_keyframeToRemove * Twc_newRefKeyframe;
    FrameNode newFrame = make_shared<FrameNODE>(FrameNODE(keyframeToRemoveId ,timestamp_keyframeToRemove,Tr_new,newRefKeyframe));
    frames.insert(make_pair(keyframeToRemoveId, newFrame));
    newRefKeyframe->addFrame(newFrame);
}

void SLAM_GRAPH::SLAMGraph::updateTwc(const FrameId& frameId,const mat4& Twc){
    if(!keyframes.count(frameId)){
        return;
    }
    KeyframeNode keyframe = keyframes.find(frameId)->second;
    keyframe->updateTwc(Twc);
}

SLAM_GRAPH::mat4 SLAM_GRAPH::SLAMGraph::getTwc(const FrameId &frameId) const{
    if(keyframes.count(frameId))
        return keyframes.find(frameId)->second->getTwc();

    if(frames.count(frameId))
        return frames.find(frameId)->second->getTwc();

    return mat4::Identity();
}

SLAM_GRAPH::Seconds SLAM_GRAPH::SLAMGraph::getTimestamp(const FrameId &frameId) const{
    if(keyframes.count(frameId))
        return keyframes.find(frameId)->second->getTimestamp();

    if(frames.count(frameId))
        return frames.find(frameId)->second->getTimestamp();

    return 0.0;
}

SLAM_GRAPH::KeyframeNode SLAM_GRAPH::SLAMGraph::getNewReferenceKeyframe(const FrameId & frameId) {
    KeyframeNode  newRefKeyframe = keyframes.begin()->second;
    int minDiff = std::abs(int(newRefKeyframe->getId()) - int(frameId));;

    for(auto& keyframe:keyframes){
        int diff = std::abs(int(keyframe.first) - int(frameId));
        if(diff < minDiff){
            minDiff = diff;
            newRefKeyframe = keyframe.second;
        }
    }

    if(newRefKeyframe->getId() == frameId){
        cout << "[SLAMGraph::getNewReferenceKeyframe]: FAILED" << endl;
        cout << "     newRefKeyframe " << newRefKeyframe->getId() << " = "<< "frameId " << frameId <<endl;
        terminate();
    }
    return newRefKeyframe;
}

void SLAM_GRAPH::SLAMGraph::correctFramesScale(const double& scale, const FrameId &keyframeId){
    if(!keyframes.count(keyframeId)){
        cout << "[SLAMGraph::correctScale]: FAILED" << endl;
        cout << "     Keyframe " << keyframeId << " doesn't exist."<<endl;
        return;
    }

    KeyframeNode keyframe = keyframes.find(keyframeId)->second;
    for(auto& frame: keyframe->getFrames()){
        frame.second->correctScale(scale, keyframe);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keyframe Node Methods
SLAM_GRAPH::KeyframeNODE::KeyframeNODE(const FrameId& id_, const Seconds& timestamp_, mat4 Twc_):
        id(id_),timestamp(timestamp_),Twc(std::move(Twc_)){}

void SLAM_GRAPH::KeyframeNODE::updateTwc(const mat4& Twc_){
    Twc = Twc_;
}

void SLAM_GRAPH::KeyframeNODE::addFrame(FrameNode& frame){
    if(frames.count(frame->getId()))
        return;
    frames.insert(make_pair(frame->getId(), frame));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame Node Methods
SLAM_GRAPH::FrameNODE::FrameNODE(const FrameId& id_, const Seconds& timestamp_,
                                 const mat4& Tr_, const KeyframeNode& refKeyframe):
        id(id_),timestamp(timestamp_){
    Tr.insert(make_pair(refKeyframe, Tr_));
}

SLAM_GRAPH::mat4 SLAM_GRAPH::FrameNODE::getTr(const KeyframeNode& keyframe) const{
    return Tr.find(keyframe)->second;
}

void SLAM_GRAPH::FrameNODE::addTr(const KeyframeNode& refkeyframe, const mat4& Tr_){
    Tr.insert(make_pair(refkeyframe, Tr_));
}

void SLAM_GRAPH::FrameNODE::removeTr(const KeyframeNode& refkeyframe){
    if(Tr.count(refkeyframe))
        Tr.erase(refkeyframe);
}

SLAM_GRAPH::mat4 SLAM_GRAPH::FrameNODE::getTwc() const{

    return Tr.begin()->first->getTwc()*Tr.begin()->second.inverse();
}

void SLAM_GRAPH::FrameNODE::correctScale(const double& scale, const KeyframeNode &refKeyframe){
    Tr.find(refKeyframe)->second.block<3,1>(0,3) *= scale;
}
