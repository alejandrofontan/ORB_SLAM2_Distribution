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
        srand (time(nullptr));
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