
cd ../Templates

./build_ORB_SLAM2.sh ORB_SLAM2_Distribution /home/alex 

resultsFolder="miniTestGroup"
./createResultsFolder.sh /home/alex/${resultsFolder}

numRuns=5
./run_SLAM.sh orbslam2 mono miniTestGroup 2023_06_23 y 1 ${numRuns} rgbdtum miniTestGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution ${resultsFolder}
./run_SLAM.sh orbslam2 mono miniTestGroup 2023_06_23 y 1 ${numRuns} kitti miniTestGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution ${resultsFolder}
./run_SLAM.sh orbslam2 mono miniTestGroup 2023_06_23 y 1 ${numRuns} eth miniTestGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution ${resultsFolder}
./run_SLAM.sh orbslam2 mono miniTestGroup 2023_06_23 y 1 ${numRuns} euroc miniTestGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution ${resultsFolder}

