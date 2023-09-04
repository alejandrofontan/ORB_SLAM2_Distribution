# Path to dataset
if [ $dataset == 'kitti' ]
then 
dataset_path="${ws_path}/KITTI"
fi

if [ $dataset == 'rgbdtum' ]
then 
dataset_path="${ws_path}/RGBD_TUM"
fi

if [ $dataset == 'nuim' ]
then 
dataset_path="${ws_path}/NUIM"
fi

if [ $dataset == 'euroc' ]
then 
dataset_path="${ws_path}/EUROC"
fi

if [ $dataset == 'monotum' ]
then 
dataset_path="${ws_path}/TUM_Mono_VO/all_sequences"
fi

if [ $dataset == 'fordva' ]
then 
dataset_path="${ws_path}/FORDVA"
fi

if [ $dataset == 'madmax' ]
then 
dataset_path="${ws_path}/MADMAX"
fi

if [ $dataset == 'oxford' ]
then 
dataset_path="${ws_path}/OXFORD"
fi

if [ $dataset == 'vector' ]
then 
dataset_path="${ws_path}/VECTOR"
fi

if [ $dataset == 'fourseasons' ]
then 
dataset_path="${ws_path}/FOURSEASONS"
fi

if [ $dataset == 'interior' ]
then 
dataset_path="${ws_path}/INTERIOR"
fi

if [ $dataset == 'eth' ]
then 
dataset_path="${ws_path}/ETH"
fi

if [ $dataset == 'tartanair' ]
then 
dataset_path="${ws_path}/TARTANAIR/mono"
fi

if [ $dataset == 'drunkards' ]
then 
dataset_path="${ws_path}/DRUNKARDS"
fi

if [ $dataset == 'rosario' ]
then 
dataset_path="${ws_path}/ROSARIO"
fi

if [ $dataset == 'canoe' ]
then 
dataset_path="${ws_path}/CANOE"
fi

if [ $dataset == 'openloris' ]
then 
dataset_path="${ws_path}/OPENLORIS"
fi

if [ $dataset == 'vkitti' ]
then 
dataset_path="${ws_path}/VKITTI"
fi
