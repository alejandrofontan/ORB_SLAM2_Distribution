//
// Created by fontan on 25/07/23.
//

#include "OrbFunctions.h"

void ORB_SLAM2::OrbFunctions::computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, const std::vector<cv::Point>& pattern)
{
    descriptors = cv::Mat::zeros((int)keypoints.size(), DESCRIPTOR_SIZE, DESCRIPTOR_MAT_TYPE);
    for (size_t i = 0; i < keypoints.size(); i++)
        OrbFunctions::computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
}

float ORB_SLAM2::OrbFunctions::IC_Angle(const cv::Mat& image, cv::Point2f pt,  const std::vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;

    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return cv::fastAtan2((float)m_01, (float)m_10);
}

void ORB_SLAM2::OrbFunctions::computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax)
{
    for(std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = OrbFunctions::IC_Angle(image, keypoint->pt, umax);
    }
}

const float factorPI = (float)(CV_PI/180.f);
void ORB_SLAM2::OrbFunctions::computeOrbDescriptor(const cv::KeyPoint& kpt,
                                 const cv::Mat& img, const cv::Point* pattern, uchar* desc)
{
    float angle = (float)kpt.angle*factorPI;
    float a = (float)cos(angle), b = (float)sin(angle);

    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    const int step = (int)img.step;

#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

#undef GET_VALUE
}

void  ORB_SLAM2::OrbFunctions::computeEndRowCircularPatch(std::vector<cv::Point>& pattern_, std::vector<int>& umax_){
    const int npoints = 512;
    const auto* pattern0 = (const cv::Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern_));

    //This is for orientation
    // pre-compute the end of a row in a circular patch
    umax_.resize(HALF_PATCH_SIZE + 1);

    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax_[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax_[v0] == umax_[v0 + 1])
            ++v0;
        umax_[v] = v0;
        ++v0;
    }
}
