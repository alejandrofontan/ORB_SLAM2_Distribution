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

#include<Definitions.h>

#include<Eigen/Dense>

using namespace std;

namespace SLAM_GRAPH {
    using FrameId = long unsigned int;
    using MapPointId = long unsigned int;
    using Seconds = double;

    class KeyframeNODE;
    class FrameNODE;
    class MapPointNODE;

    typedef shared_ptr<KeyframeNODE> KeyframeNode;
    typedef shared_ptr<FrameNODE> FrameNode;
    typedef shared_ptr<MapPointNODE> MapPointNode;

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
    class MapPointNODE {

    private:
        MapPointId id{};
        vec3 XYZ{};

    public:
        MapPointNODE(const MapPointId & id_, const vec3& XYZ_);
        void updateXYZ(const vec3& XYZ_);
        [[nodiscard]] MapPointId getId() const{return id;};
        [[nodiscard]] vec3 getXYZ() const{return XYZ;};
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class SLAMGraph {
    public:
        enum VerbosityLevel{
            NONE = 0,
            LOW = 1,
            MEDIUM = 2,
            HIGH = 3
        };

        SLAMGraph(const VerbosityLevel& verbosity): verbosity(verbosity){};
        void initialize(const FrameId& id_1, const Seconds& timestamp_1, const mat4& Twc_1,
                        const FrameId& id_2, const Seconds& timestamp_2, const mat4& Twc_2);

        void addKeyframe(const FrameId& keyframeId, const Seconds& timestamp, const mat4& Twc);
        void addFrame(const FrameId &frameId, const Seconds& timestamp, const mat4& Tcw, const FrameId& refKeyframeId);
        void addMapPoint(const MapPointId& mapPointId, const vec3& XYZ);
        void removeKeyframe(const FrameId& keyframeToRemoveId);
        void updateTwc(const FrameId& frameId,const mat4& Twc);
        void correctFramesScale(const double& scale, const FrameId &keyframeId);

        KeyframeNode getNewReferenceKeyframe(const FrameId & frameId);

        map<FrameId,shared_ptr<FrameNODE>> const * getFrames(){return &frames;};
        map<FrameId,shared_ptr<KeyframeNODE>> const * getKeyframes(){return &keyframes;};
        set<FrameId> getIds(){return ids;};

        [[nodiscard]] mat4 getTwc(const FrameId &frameId) const;
        [[nodiscard]] mat4 getTcw(const FrameId &frameId) const;
        [[nodiscard]] Seconds getTimestamp(const FrameId &frameId) const;
        [[nodiscard]] vec3 getXYZ(const MapPointId &mapPointId) const;

        void saveMap();
        void resetMapFromCopy();
        void addNoiseToSavedMap(const double& noise_);
        double getMeanKeyframeDistance();

    private:
        VerbosityLevel verbosity{};
        map<FrameId,shared_ptr<KeyframeNODE>> keyframes{};
        map<FrameId,shared_ptr<FrameNODE>> frames{};
        map<FrameId,shared_ptr<MapPointNODE>> mapPoints{};

        set<FrameId> ids{};

        map<FrameId,mat4> keyframeTwc_0{};
        map<FrameId,vec3> mapPointXYZ_0{};
    };
    typedef shared_ptr<SLAMGraph> SlamGraph;
}

#endif //ORB_SLAM2_DETERMINISTIC_SLAMGRAPH_H
