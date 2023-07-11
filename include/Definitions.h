//
// Created by fontan on 22/06/23.
//

#ifndef ORB_SLAM2_DISTRIBUTION_DEFINITIONS_H
#define ORB_SLAM2_DISTRIBUTION_DEFINITIONS_H

#include<Eigen/Dense>

#define DESCRIPTOR_TYPE ORB
#define DESCRIPTOR_DISTANCE_TYPE int
#define DESCRIPTOR_DISTANCE_FUNCTION cv::NORM_HAMMING
#define DESCRIPTOR_MAT_TYPE CV_8UC1
#define DESCRIPTOR_SIZE 32

//#define DESCRIPTOR_TYPE SUPERPOINT
//#define DESCRIPTOR_DISTANCE_TYPE float
//#define DESCRIPTOR_DISTANCE_FUNCTION cv::NORM_L2
//#define DESCRIPTOR_MAT_TYPE CV_32FC1
//#define DESCRIPTOR_SIZE 32

typedef double dataType;
typedef Eigen::Matrix<dataType,3,1> vec3;
typedef Eigen::Matrix<dataType,3,3> mat3;
typedef Eigen::Matrix<dataType,4,4> mat4;
typedef Eigen::Quaterniond quat;

#endif //ORB_SLAM2_DISTRIBUTION_DEFINITIONS_H
