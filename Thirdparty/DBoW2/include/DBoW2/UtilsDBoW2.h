//
// Created by fontan on 24/07/23.
//

#ifndef UTILSDBOW2_H
#define UTILSDBOW2_H

#include <iostream>
#include <cmath>

void showProgress(const int& i, const int& N, const std::string& message = ""){

    double progress = (double)i/(double)N*100;
    if(abs(progress/10.0-int(progress/10.0)) < 0.001)
        std::cout << "["<< message <<  "] Progress (" << i <<"/"<< N << ") : " << int(progress) << "%" << std::endl ;
}

#endif //ORB_SLAM2_DISTRIBUTION_UTILSDBOW2_H
