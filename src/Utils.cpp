//
// Created by fontan on 9/06/23.
//

#include "Utils.h"
#include <ctime>
#include <random>
#include <algorithm>

bool ORB_SLAM2::RandomIntegerGenerator::need_to_be_seeded = true;
int ORB_SLAM2::RandomIntegerGenerator::randomIntegerIndex = -1;
std::vector<int> ORB_SLAM2::RandomIntegerGenerator::listOfRandomIntegers{};

std::string ORB_SLAM2::paddingZeros(const std::string& number, const size_t numberOfZeros){
    std::string zeros{};
    for(size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return (zeros + number);
}

void ORB_SLAM2::RandomIntegerGenerator::seedRandomGenerator(){
#ifdef COMPILED_DETERMINISTIC
    if(need_to_be_seeded){
        srand (0);
        need_to_be_seeded = false;
        listOfRandomIntegers.reserve(amountOfRandomIntegers);
        for(int iRand{}; iRand < amountOfRandomIntegers; iRand++)
            listOfRandomIntegers.push_back(rand());
    }
#else
    if(need_to_be_seeded){
        auto seed = time(nullptr);
        srand (seed);
        need_to_be_seeded = false;
    }
#endif
}

int ORB_SLAM2::RandomIntegerGenerator::getRandomNumber(){
    seedRandomGenerator();
#ifdef COMPILED_DETERMINISTIC
    randomIntegerIndex++;
    if(randomIntegerIndex >= listOfRandomIntegers.size()){
        std::shuffle ( listOfRandomIntegers.begin(), listOfRandomIntegers.end(), std::default_random_engine(0));
        randomIntegerIndex = 0;
    }
    return listOfRandomIntegers[randomIntegerIndex];
#else
    return rand();
#endif
}

int ORB_SLAM2::RandomIntegerGenerator::getRandomNumber(const int& maxNumber) {

    return int((((double)getRandomNumber())/((double)RAND_MAX)) * maxNumber) ;
}

void ORB_SLAM2::saveVectorToFile(std::vector<double>& vectorToSave, const std::string& file, const int& precision){
    std::ofstream f;
    f.open(file.c_str());
    f << std::fixed;

    for (auto &element: vectorToSave)
        f << std::setprecision(precision) << element << std::endl;
}

void ORB_SLAM2::vectorMedian(float& median, const std::vector<float>& v){
    if(v.empty())
    {
        median = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    std::vector<float> vOrdered = v;
    std::sort(vOrdered.begin(),vOrdered.end());
    int center = (int) (0.5f * float(vOrdered.size()));
    median = vOrdered[center];
}

void ORB_SLAM2::vectorMean(float& mean, const std::vector<float>& v){
    if(v.empty())
    {
        mean = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    mean = std::accumulate(v.begin(), v.end(), 0.0) / (static_cast<float>(v.size()));
}

void ORB_SLAM2::vectorStd(float& std, const float& mean, const std::vector<float>& v){
    if(v.size() < 2)
    {
        std = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    float sum_of_squared_deviations = std::accumulate(v.begin(), v.end(), 0.0,
                                                         [&mean](float accumulator, float data_point) {
                                                             float deviation = data_point - mean;
                                                             return accumulator + deviation * deviation;
                                                         });
    std = sum_of_squared_deviations / (static_cast<float>(v.size() - 1));
}

void ORB_SLAM2::vectorPercentage(float& th, const float& percentage, const std::vector<float>& v){
    if(v.empty())
    {
        th = std::numeric_limits<float>::quiet_NaN();
        return;
    }
    std::vector<float> vOrdered = v;
    std::sort(vOrdered.begin(),vOrdered.end());
    int position = (int) (percentage * float(vOrdered.size()));
    th = vOrdered[position];
}

void printMessage(const std::string& function, const std::string& message,
                             const VerbosityLevel& verbosityLevel, const VerbosityLevel& verbosityLevelRequired){
    if(verbosityLevel >= verbosityLevelRequired)
        std::cout << "[" << function << "] : " <<  message << std::endl;
}