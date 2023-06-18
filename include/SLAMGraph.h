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

    class KeyframeNODE;
    class FrameNODE;
    typedef shared_ptr<KeyframeNODE> KeyframeNode;
    typedef shared_ptr<FrameNODE> FrameNode;

    class KeyframeNODE {
    private:
        FrameId id{};
        Seconds timestamp{};
        mat4 Twc{mat4::Identity()};
        map<FrameId,FrameNode> frames{};

    public:
        KeyframeNODE(const FrameId& id_, const Seconds& timestamp_, mat4  Twc_);

        void updateTwc(const mat4& Twc_);
        void addFrame(FrameNode& frame);

        [[nodiscard]] map<FrameId,FrameNode> getFrames()const {return frames;};
        [[nodiscard]] mat4 getTwc() const{return Twc;};
        [[nodiscard]] mat4 getTcw() const{return Twc.inverse();};
        [[nodiscard]] Seconds getTimestamp() const{return timestamp;};
        [[nodiscard]] FrameId getId() const{return id;};
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class FrameNODE {

    private:
        FrameId id{};
        Seconds timestamp{};
        unordered_map<KeyframeNode, mat4> Tr{}; // Relative Transformations

    public:
        FrameNODE(const FrameId& id_, const Seconds& timestamp_, const mat4& Tr_, const KeyframeNode& refKeyframe);

        void addTr(const KeyframeNode& refkeyframe,  const mat4& Tr_);
        void removeTr(const KeyframeNode& refkeyframe);
        void correctScale(const double& scale, const KeyframeNode &refKeyframe);

        [[nodiscard]] Seconds getTimestamp() const{return timestamp;};
        [[nodiscard]] FrameId getId() const{return id;};
        [[nodiscard]] size_t amountOfRelativePoses() const{return Tr.size();};
        [[nodiscard]] mat4 getTr(const KeyframeNode& keyframe) const;
        [[nodiscard]] mat4 getTwc() const;
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class SLAMGraph {
    public:
        SLAMGraph() = default;
        void initialize(const FrameId& id_1, const Seconds& timestamp_1, const mat4& Twc_1,
                        const FrameId& id_2, const Seconds& timestamp_2, const mat4& Twc_2);

        void addKeyframe(const FrameId& keyframeId, const Seconds& timestamp, const mat4& Twc);
        void addFrame(const FrameId &frameId, const Seconds& timestamp, const mat4& Tcw, const FrameId& refKeyframeId);
        void removeKeyframe(const FrameId& keyframeToRemoveId);
        void updateTwc(const FrameId& frameId,const mat4& Twc);
        void correctFramesScale(const double& scale, const FrameId &keyframeId);

        KeyframeNode getNewReferenceKeyframe(const FrameId & frameId);

        map<FrameId,shared_ptr<FrameNODE>> const * getFrames(){return &frames;};
        map<FrameId,shared_ptr<KeyframeNODE>> const * getKeyframes(){return &keyframes;};
        set<FrameId> getIds(){return ids;};

        [[nodiscard]] mat4 getTwc(const FrameId &frameId) const;
        [[nodiscard]] Seconds getTimestamp(const FrameId &frameId) const;

    private:
        map<FrameId,shared_ptr<KeyframeNODE>> keyframes{};
        map<FrameId,shared_ptr<FrameNODE>> frames{};
        set<FrameId> ids{};

    };
    typedef shared_ptr<SLAMGraph> SlamGraph;
}

#endif //ORB_SLAM2_DETERMINISTIC_SLAMGRAPH_H
