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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <Definitions.h>

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};



class FeatureExtractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    static DescriptorType descriptorType;

    FeatureExtractor(int nfeatures, float scaleFactor, int nlevels,
                     int iniThFAST, int minThFAST);

    ~FeatureExtractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
#ifdef ORB_FEATURE
        void operator()( cv::InputArray _image, cv::InputArray _mask, std::vector<cv::KeyPoint>& _keypoints,cv::OutputArray _descriptors, std::vector<mat2>& keyPointsInformation);
#else
    void operator()( cv::InputArray image, cv::InputArray mask,
                     std::vector<cv::KeyPoint>& keypoints,
                     cv::Mat& descriptors, std::vector<mat2>& keyPointsInformation);
#endif

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void estimateDesiredNumberOfFeaturesPerLevel();
    static void createDetector(cv::Ptr<DETECTOR_CV>& detector);
    static void updateDetectorSettings(cv::Ptr<DETECTOR_CV>& detector, float factor = 2.0);
    mat2 estimateKeyPointInformation(cv::KeyPoint& pt);
    void ComputePyramid(cv::Mat image);

    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    void ComputeKeyPointsAndDescriptors(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, std::vector<mat2>& keyPointsInformation);

    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    std::vector<cv::Point> pattern;
    std::vector<int> umax;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    //std::vector<float> mvLevelSigma2;
    //std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

