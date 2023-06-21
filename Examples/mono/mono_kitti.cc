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
*
* This file has been modified from the original version in ORB-SLAM2
* by Alejandro Fontán Villacampa (June 2023 Queensland University Of Technology).
* For more information see <https://github.com/alejandrofontan/ORB_SLAM2_Deterministic>
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &pathToSequence, vector<string> &imageFilenames, vector<ORB_SLAM2::Seconds> &timestamps);

int main(int argc, char **argv)
{
    if(argc != 8)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence "
                        "path_to_output experimentIndex activateVisualization runBackwards" << endl;
        return 1;
    }

    // ORB_SLAM2 inputs
    string path_to_vocabulary = string(argv[1]);
    string path_to_settings = string(argv[2]);
    string path_to_sequence = string(argv[3]);
    string path_to_output = string(argv[4]);
    string experimentIndex = string(argv[5]);
    bool activateVisualization = bool(std::stoi(string(argv[6])));
    bool runBackwards = bool(std::stoi(string(argv[7])));
    string resultsPath = path_to_output + "/" + ORB_SLAM2::paddingZeros(experimentIndex);

    // Retrieve paths to images
    vector<string> imageFilenames{};
    vector<ORB_SLAM2::Seconds> timestamps{};
    LoadImages(path_to_sequence, imageFilenames, timestamps);

    size_t nImages = imageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary,path_to_settings,
                           ORB_SLAM2::System::MONOCULAR,
                           stoi(experimentIndex),
                           activateVisualization);

    // Vector for tracking time statistics
    vector<ORB_SLAM2::Seconds> timesTrack{};
    timesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(size_t ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        SLAM.readImage(im, imageFilenames[ni]);
        ORB_SLAM2::Seconds tframe = timestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << imageFilenames[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        ORB_SLAM2::Seconds ttrack= std::chrono::duration_cast<std::chrono::duration<ORB_SLAM2::Seconds> >(t2 - t1).count();

        timesTrack[ni] = ttrack;

        // Wait to load the next frame
        ORB_SLAM2::Seconds T = 0;
        if(ni < nImages-1)
            T = timestamps[ni+1] - tframe;
        else if(ni>0)
            T = tframe - timestamps[ni-1];

#ifndef COMPILED_SEQUENTIAL
        if(ttrack < T)
            usleep((T-ttrack)*1e6);
#endif
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(timesTrack.begin(),timesTrack.end());
    ORB_SLAM2::Seconds totaltime = 0;
    for(size_t ni = 0; ni < nImages; ni++)
    {
        totaltime += timesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << timesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    //
    SLAM.SaveStatisticsToFiles(resultsPath + "_");

    // Save camera trajectory
    SLAM.SaveFrameTrajectoryTUM(resultsPath + "_FrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(resultsPath + "_KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &pathToSequence, vector<string> &imageFilenames, vector<ORB_SLAM2::Seconds> &timestamps)
{
    ifstream times;
    string pathToTimeFile = pathToSequence + "/times.txt";
    times.open(pathToTimeFile.c_str());

    while(!times.eof())
    {
        string s;
        getline(times,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            ORB_SLAM2::Seconds t;
            ss >> t;
            timestamps.push_back(t);
        }
    }

    string prefixLeft = pathToSequence + "/image_0/";

    const size_t nTimes = timestamps.size();
    imageFilenames.resize(nTimes);

    for(size_t i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        imageFilenames[i] = prefixLeft + ss.str() + ".png";
    }
}
