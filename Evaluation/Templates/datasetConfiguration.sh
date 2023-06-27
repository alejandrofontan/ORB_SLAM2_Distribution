

if [ $dataset == 'kitti' ]
then
	groundtruth_file="$dataset_path/data_odometry_poses/dataset/poses/${sequenceName}_TUM_format.txt"
	max_diff=0.1 # 10Hz
	frequence=10
fi

if [ $dataset == 'rgbdtum' ]
then
	groundtruth_file="$dataset_path/${sequenceName}/groundtruth.txt"
	max_diff=0.033 # 30Hz
	frequence=30
fi
				
if [ $dataset == 'euroc' ]
then
	groundtruth_file="${ws_path}/TUM_Mono_VO/supp_v2/gtFiles/new_mav2_${sequenceName}.txt"
	max_diff=0.05 # 20Hz
	frequence=20
fi
				
if [ $dataset == 'monotum' ]
then
	groundtruth_file="${ws_path}/TUM_Mono_VO/supp_v2/gtFiles/new_${sequenceName}.txt"
	max_diff=0.025 # 40Hz
	frequence=40
fi

if [ $dataset == 'fordva' ]
then
	groundtruth_file="$dataset_path/${sequenceName}/new_pose_ground_truth.txt" #???????????????
	max_diff=0.067 # 15Hz
	frequence=15
fi
				
if [ $dataset == 'madmax' ]
then
	groundtruth_file="$dataset_path/${sequenceName}/groundtruth.txt" #???????????????
	max_diff=0.067 # 30Hz
	frequence=15
fi

if [ $dataset == 'oxford' ]
then
	groundtruth_file="$dataset_path/${sequenceName}/groundtruth.txt" #???????????????
	max_diff=0.033 # 30Hz
	frequence=30
fi

if [ $dataset == 'vector' ]
then
	groundtruth_file="$dataset_path/${sequenceName}/groundtruth.txt"
	max_diff=0.033 # 30Hz
	frequence=30
fi

if [ $dataset == 'fourseasons' ]
then
	groundtruth_file="$dataset_path/${sequenceName}/groundtruth.txt" #???????????????
	max_diff=0.033 # 30Hz
	frequence=30
fi
