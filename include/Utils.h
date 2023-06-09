//
// Created by fontan on 9/06/23.
//

#ifndef ORB_SLAM2_DETERMINISTIC_UTILS_H
#define ORB_SLAM2_DETERMINISTIC_UTILS_H

#include <vector>
#include <iostream>

using FrameId = long unsigned int;
using KeyframeId = long unsigned int;
using KeypointIndex = int;
using OctaveType = int;

namespace ORB_SLAM2
{
    class RandomIntegerGenerator
    {
    private:
        static bool need_to_be_seeded;
        static int randomIntegerIndex;
        static std::vector<int> listOfRandomIntegers;
        static const int amountOfRandomIntegers{5000};
    public:
        static void seedRandomGenerator();
        static int getRandomNumber();
        static int getRandomNumber(const int& maxNumber);

    };

}

#endif //ORB_SLAM2_DETERMINISTIC_UTILS_H
