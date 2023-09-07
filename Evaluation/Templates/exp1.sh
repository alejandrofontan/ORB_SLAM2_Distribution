
exp="orb_gamma"
numRuns=30
./build_ORB_SLAM2.sh ORB_SLAM2_Distribution /home/alex 

rm -rf "/home/alex/${exp}"
./createResultsFolder.sh "/home/alex/${exp}"

./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg3_structure_texture_far 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
./run_SLAM.sh orbslam2 mono ${exp}  2023_08_31 y 1 ${numRuns} kitti 04 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} eth table_3 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}

#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} eth einstein_1 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg3_long_office_household 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg1_desk 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg2_desk 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} nuim traj2_frei_png 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} euroc V2_01_easy 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} kitti 03 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg2_xyz 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}

#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} kitti 03 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} kitti 04 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg3_structure_texture_far 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg1_xyz 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} kitti 06 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} euroc V2_01_easy 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg2_desk 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_08_31 y 1 ${numRuns} monotum sequence_23 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} nuim traj0_frei_png 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} tartanair ME004 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} drunkards 00000_level0 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} rosario sequence01 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} rgbdtum rgbd_dataset_freiburg1_desk 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} eth einstein_1 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}

#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} openloris cafe1-2 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
#./run_SLAM.sh orbslam2 mono ${exp} 2023_06_23 y 1 ${numRuns} vkitti Scene01_morning 0 /home/alex vocabulary ORB_SLAM2_Distribution ${exp}
