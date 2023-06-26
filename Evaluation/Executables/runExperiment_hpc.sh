

experimentName_="test1"
systemVersion_="ORB_SLAM2_Distribution"
datasets=("rgbdtum" "kitti" "euroc")
sequenceGroups=("testGroup" "testGroup" "testGroup")	

workspacePath_="/home/fontan"
resultsFolder_="ORB_SLAM2_Ablation"

experimentDate=$(date +'%Y_%m_%d')
experimentComments="No comment"

echo "Run Experiment HPC: ${experimentName_}"
echo "    systemVersion_: ${systemVersion_}"

#../Templates/build_ORB_SLAM2_interactiveJob.sh ${systemVersion_} ${workspacePath_}

cd ../Templates
./createResultsFolder.sh "${workspacePath_}/${resultsFolder_}/${experimentName_}"
cd ../Executables

iDataset=0
jobIndex=0
for dataset in "${datasets[@]}"
do
	sequenceGroup=${sequenceGroups[iDataset]}
	cd ../Templates
	. ./getSequences.sh
	cd ../Executables


	for sequenceName in "${sequenceNames[@]}"
	do
		echo "${sequenceName}_job_${jobIndex}"

		qsub -N "${experimentName_}_job_${jobIndex}_${sequenceName}" -v experimentName=${experimentName_},resultsFolder="${resultsFolder_}/${experimentName_}",systemVersion=${systemVersion_},experimentDate=${experimentDate},dataset=${dataset},sequenceGroup=${sequenceName},workspacePath=${workspacePath_},experimentComments="${experimentComments}" experiment_SLAM_hpc.sh
	
		jobIndex=$((jobIndex + 1))
	done

	iDataset=$((iDataset + 1))
done







