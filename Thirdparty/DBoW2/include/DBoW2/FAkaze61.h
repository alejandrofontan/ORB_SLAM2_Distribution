/**
 * File: FAkaze61.h
 * Date: July 2023
 * Author: Dorian Galvez-Lopez (Modified by Alejandro Fontan Villacampa)
 * Description: functions for Akaze61 descriptors
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_F_AKAZE61__
#define __D_T_F_AKAZE61__

#include <opencv2/core.hpp>
#include <vector>
#include <string>

#include "FClass.h"

#define DESCRIPTOR_CLASS_NAME FAkaze61
#define DESCRIPTOR_FORMAT cv::Mat
#define DESCRIPTOR_LENGTH 61

namespace DBoW2 {

/// Functions to manipulate ORB descriptors
    class DESCRIPTOR_CLASS_NAME: protected FClass
    {
    public:

        /// Descriptor type
        typedef DESCRIPTOR_FORMAT TDescriptor;
        /// Pointer to a single descriptor
        typedef const TDescriptor *pDescriptor;
        /// Descriptor length (in bytes)
        static const int L = DESCRIPTOR_LENGTH;

        /**
         * Calculates the mean value of a set of descriptors
         * @param descriptors
         * @param mean mean descriptor
         */
        static void meanValue(const std::vector<pDescriptor> &descriptors,
                              TDescriptor &mean);

        /**
         * Calculates the distance between two descriptors
         * @param a
         * @param b
         * @return distance
         */
        static double distance(const TDescriptor &a, const TDescriptor &b);

        /**
         * Returns a string version of the descriptor
         * @param a descriptor
         * @return string version
         */
        static std::string toString(const TDescriptor &a);

        /**
         * Returns a descriptor from a string
         * @param a descriptor
         * @param s string version
         */
        static void fromString(TDescriptor &a, const std::string &s);

        /**
         * Returns a mat with the descriptors in float format
         * @param descriptors
         * @param mat (out) NxL 32F matrix
         */
        static void toMat32F(const std::vector<TDescriptor> &descriptors,
                             cv::Mat &mat);

        /**
         * Returns a mat with the descriptors in float format
         * @param descriptors NxL CV_8U matrix
         * @param mat (out) NxL 32F matrix
         */
        static void toMat32F(const cv::Mat &descriptors, cv::Mat &mat);

        /**
         * Returns a matrix with the descriptor in OpenCV format
         * @param descriptors vector of N row descriptors
         * @param mat (out) NxL CV_8U matrix
         */
        static void toMat8U(const std::vector<TDescriptor> &descriptors,
                            cv::Mat &mat);

    };

} // namespace DBoW2

#endif


