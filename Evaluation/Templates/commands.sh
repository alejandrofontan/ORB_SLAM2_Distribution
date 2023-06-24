


./build_ORB_SLAM2.sh ORB_SLAM2_Distribution /home/alex 

#./build_ORB_SLAM2_fast.sh ORB_SLAM2_Distribution /home/alex 

./createResultsFolder.sh /home/alex/secondAblation

./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 7 kitti testGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution secondAblation
./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 7 rgbdtum testGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution secondAblation
./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 7 euroc testGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution secondAblation
./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 7 monotum testGroup 0 /home/alex ORBvoc.txt ORB_SLAM2_Distribution secondAblation



