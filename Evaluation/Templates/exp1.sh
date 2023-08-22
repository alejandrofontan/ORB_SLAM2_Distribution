
exp="GBA_orb"
numRuns=1
#rm -rf "/home/alex/${exp}"

./build_ORB_SLAM2_fast.sh ORB_SLAM2_Distribution /home/alex 

./createResultsFolder.sh "/home/alex/${exp}"

#./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} eth table_3 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} kitti 04 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg3_structure_texture_far 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg1_xyz 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} kitti 06 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} euroc V2_01_easy 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono test1 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg2_desk 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
