//
// Created by fontan on 22/06/23.
//

#ifndef ORB_SLAM2_DISTRIBUTION_DEFINITIONS_H
#define ORB_SLAM2_DISTRIBUTION_DEFINITIONS_H

#include<Eigen/Dense>

namespace ORB_SLAM2 {
    enum DescriptorType {
        ORB = 0,
        SUPERPOINT = 1,
        AKAZE = 2,
        SIFT = 3,
        BRISK = 4,
        KAZE = 5,
        SURF = 6,
        BRIEF = 7
    };
}

#define ORB_FEATURE

#ifdef ORB_FEATURE
#define DESCRIPTOR_NAME "orb"
#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::ORB
#define DESCRIPTOR_SIZE 32
#define DETECTOR_CV cv::ORB
#define DESCRIPTOR_CV cv::ORB
#define DBOW_SRC_F "Thirdparty/DBoW2/include/DBoW2/FORB.h"
#define DESCRIPTOR_F FORB
#define VOCABULARY_FILE "orb_DBoW2_voc.yml"
#define BINARY_DESCRIPTOR
#endif

#ifdef AKAZE_FEATURE
#define DESCRIPTOR_NAME "akaze"
#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::AKAZE
#define DESCRIPTOR_SIZE 61
#define DETECTOR_CV cv::AKAZE
#define DESCRIPTOR_CV cv::AKAZE
#define DBOW_SRC_F "Thirdparty/DBoW2/include/DBoW2/FAKAZE.h"
#define DESCRIPTOR_F FAKAZE
#define VOCABULARY_FILE "akaze_DBoW2_voc.yml"
#define BINARY_DESCRIPTOR
#endif

#ifdef BRIEF_FEATURE
#define DESCRIPTOR_NAME "brief"
#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::BRIEF
#define DESCRIPTOR_SIZE 32
#define DETECTOR_CV cv::xfeatures2d::StarDetector
#define DESCRIPTOR_CV cv::xfeatures2d::BriefDescriptorExtractor
#define DBOW_SRC_F "Thirdparty/DBoW2/include/DBoW2/FBrief.h"
#define DESCRIPTOR_F FBrief
#define VOCABULARY_FILE "brief_DBoW2_voc.yml"
#define BINARY_DESCRIPTOR
#endif

#ifdef  KAZE_FEATURE
#define DESCRIPTOR_NAME "kaze"
#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::KAZE
#define DESCRIPTOR_SIZE 64
#define DETECTOR_CV cv::KAZE
#define DESCRIPTOR_CV cv::KAZE
#define DBOW_SRC_F "Thirdparty/DBoW2/include/DBoW2/FKAZE.h"
#define DESCRIPTOR_F FKAZE
#define VOCABULARY_FILE "kaze_DBoW2_voc.yml"
#endif

#ifdef SURF_FEATURE
#define DESCRIPTOR_NAME "surf"
#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::SURF
#define DESCRIPTOR_SIZE 64
#define DETECTOR_CV cv::xfeatures2d::SURF
#define DESCRIPTOR_CV cv::xfeatures2d::SURF
#define DBOW_SRC_F "Thirdparty/DBoW2/include/DBoW2/FSurf64.h"
#define DESCRIPTOR_F FSurf64
#define VOCABULARY_FILE "surf_DBoW2_voc.yml"
#endif

#ifdef SIFT_FEATURE
#define DESCRIPTOR_NAME "sift"
#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::SIFT
#define DESCRIPTOR_SIZE 128
#define DETECTOR_CV cv::SIFT
#define DESCRIPTOR_CV cv::SIFT
#define DBOW_SRC_F "Thirdparty/DBoW2/include/DBoW2/FSift.h"
#define DESCRIPTOR_F FSift
#define VOCABULARY_FILE "sift_DBoW2_voc.yml"
#endif

#ifdef BINARY_DESCRIPTOR
    #define DESCRIPTOR_DISTANCE_TYPE int
    #define DESCRIPTOR_DISTANCE_FUNCTION cv::NORM_HAMMING
    #define DESCRIPTOR_MAT_TYPE CV_8UC1
    #define DESCRIPTOR_FORMAT cv::Mat
#else
    #define DESCRIPTOR_DISTANCE_TYPE float
    #define DESCRIPTOR_DISTANCE_FUNCTION cv::NORM_L2SQR
    #define DESCRIPTOR_MAT_TYPE CV_32F
    #define DESCRIPTOR_FORMAT std::vector<float>
#endif

//#define DESCRIPTOR_TYPE ORB_SLAM2::DescriptorType::BRISK
//#define DESCRIPTOR_DISTANCE_TYPE int
//#define DESCRIPTOR_DISTANCE_FUNCTION cv::NORM_HAMMING
//#define DESCRIPTOR_MAT_TYPE CV_8UC1
//#define DESCRIPTOR_SIZE 64
//#define DESCRIPTOR_CV cv::BRISK


//#define DESCRIPTOR_TYPE SUPERPOINT
//#define DESCRIPTOR_DISTANCE_TYPE float
//#define DESCRIPTOR_DISTANCE_FUNCTION cv::NORM_L2
//#define DESCRIPTOR_MAT_TYPE CV_32FC1
//#define DESCRIPTOR_SIZE 32

#ifdef COMPILED_WITH_DBOW2
#define DBOW DBoW2
#define DBOW_SRC_BOWVECTOR "Thirdparty/DBoW2/include/DBoW2/BowVector.h"
#define DBOW_SRC_FEATUREVECTOR "Thirdparty/DBoW2/include/DBoW2/FeatureVector.h"
#define DBOW_SRC_TEMPLATEDVOCABULARY "Thirdparty/DBoW2/include/DBoW2/TemplatedVocabulary.h"
#endif

#ifdef COMPILED_WITH_DBOW3
#define DBOW DBoW3
#define DBOW_SRC_BOWVECTOR "Thirdparty/DBow3/src/BowVector.h"
#define DBOW_SRC_FEATUREVECTOR "Thirdparty/DBow3/src/FeatureVector.h"
#define DBOW_SRC_RANDOM "Thirdparty/DBow3/src/Random.h"
#define DBOW_SRC_DBOW3 "Thirdparty/DBow3/src/DBoW3.h"
#endif

typedef double dataType;
typedef Eigen::Matrix<dataType,3,1> vec3;
typedef Eigen::Matrix<dataType,3,3> mat3;
typedef Eigen::Matrix<dataType,4,4> mat4;
typedef Eigen::Quaterniond quat;

#endif //ORB_SLAM2_DISTRIBUTION_DEFINITIONS_H
