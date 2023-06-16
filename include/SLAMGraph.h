//
// Created by fontan on 15/06/23.
//

#ifndef ORB_SLAM2_DETERMINISTIC_SLAMGRAPH_H
#define ORB_SLAM2_DETERMINISTIC_SLAMGRAPH_H

#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <unordered_map>
#include <set>
#include<mutex>

#include<Eigen/Dense>

using namespace std;



namespace SLAM_GRAPH {
    using FrameId = long unsigned int;
    using Seconds = double;

    typedef double dataType;
    typedef Eigen::Matrix<dataType,3,1> vec3;
    typedef Eigen::Matrix<dataType,3,3> mat3;
    typedef Eigen::Matrix<dataType,4,4> mat4;
    typedef Eigen::Quaterniond quat;

    class KeyframeNODE {
    private:
        FrameId frameId{};
        Seconds timestamp{};
        mat4 Twc{};
        map<FrameId,shared_ptr<KeyframeNODE>> covisibleKeyframes{};

    public:
        KeyframeNODE(const FrameId& frameId_, const Seconds& timestamp_, const mat4& Twc_);
        void addCovisibleKeyframe(const shared_ptr<KeyframeNODE>& covKeyframe);
        void removeCovisibleKeyframe(const FrameId& covKeyframeId, const shared_ptr<KeyframeNODE>& auxKeyframe);
        shared_ptr<KeyframeNODE> getAuxiliaryCovisibleKeyframe();
        map<FrameId,shared_ptr<KeyframeNODE>> getCovisibleKeyframes(){return covisibleKeyframes;};
        void updatePose(const mat4& Twc_);
        [[nodiscard]] mat4 getAbsolutePose() const{return Twc;};
        [[nodiscard]] Seconds getTimestamp() const{return timestamp;};

        //void addFrame(const FrameId& frameId);
        //void addCovisibleKeyframe(const shared_ptr<KeyframeNode>& covKeyframe);
        //shared_ptr<KeyframeNode> getOldestCovisibleKeyframe();
        //set<FrameId> frames{};
    };
    typedef shared_ptr<KeyframeNODE> KeyframeNode;

    class FrameNODE {

    public:
        FrameId frameId{};
        Seconds timestamp{};
        bool nodeIsKeyframe{false};
        unordered_map<shared_ptr<KeyframeNODE>, vector<mat4>> Tr{}; // Relative Transformations

    public:
        FrameNODE(const FrameId& frameId, const Seconds& timestamp, const mat4& Tr_, const shared_ptr<KeyframeNODE>& refKeyframe);
        //void convertToKeyframe(const shared_ptr<KeyframeNode>& refKeyframe);
        bool isKeyframe() const;
    };
    typedef shared_ptr<FrameNODE> FrameNode;

    class SLAMGraph {
    public:
        SLAMGraph() = default;
        void initialize(const FrameId& frameId_1, const Seconds& timestamp_1, const mat4& Twc_1,
                        const FrameId& frameId_2, const Seconds& timestamp_2, const mat4& Twc_2);
        void addKeyframe(const FrameId& frameId, const Seconds& timestamp, const mat4& Twc, const FrameId& refKeyframeId);
        void removeKeyframe(const FrameId& keyframeToRemoveId);
        void updateKeyframePose(const FrameId& frameId,const mat4& Twc);

        map<FrameId,shared_ptr<KeyframeNODE>> const * getKeyframes(){return &keyframes;};
        //void addFrame(const FrameId &frameId, const Seconds& timestamp, const mat4& Tr_, const FrameId& refKeyframeId);

    private:
        std::mutex mutexSlamGraph;
        map<FrameId,shared_ptr<KeyframeNODE>> keyframes{};
        map<FrameId,FrameNODE> frames{};

        static void linkKeyframes(KeyframeNode& keyframe1, KeyframeNode& keyframe2);

    };
    typedef shared_ptr<SLAMGraph> SlamGraph;
}

#endif //ORB_SLAM2_DETERMINISTIC_SLAMGRAPH_H
