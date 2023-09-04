if [ $dataset == 'kitti' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/data_odometry_gray/dataset/sequences/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > $system_output 2>&1
fi

if [ $dataset == 'rgbdtum' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi

if [ $dataset == 'nuim' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi
	
if [ $dataset == 'euroc' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings} ${dataset_path}/${sequenceName}/mav0/cam0/data  ${sequenceFolder} ${expId} ${activeVis} ${backwards} Examples/mono/EuRoC_TimeStamps/${sequenceTimestamps[seqIndex]}.txt > $system_output 2>&1
fi		

if [ $dataset == 'monotum' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} "${dataset_path}/${sequenceName}" ${sequenceFolder} ${expId} ${activeVis} ${backwards}  > $system_output 2>&1				
fi	

if [ $dataset == 'fordva' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > $system_output 2>&1
fi		

if [ $dataset == 'madmax' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi	
				
if [ $dataset == 'oxford' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi
				
if [ $dataset == 'vector' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi	
				
if [ $dataset == 'fourseasons' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi
				
if [ $dataset == 'eth' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi
				
if [ $dataset == 'interior' ]
then
	./${executable} Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi
				
if [ $dataset == 'tartanair' ] || [ $dataset == 'drunkards' ] || [ $dataset == 'rosario' ] || [ $dataset == 'openloris' ] || [ $dataset == 'vkitti' ]
then
	./Examples/${mode}/${mode}_template Vocabulary/$voc Examples/mono/${sequenceSettings[seqIndex]} ${dataset_path}/${sequenceName} ${sequenceFolder} ${expId} ${activeVis} ${backwards} > /dev/null > $system_output 2>&1
fi				
