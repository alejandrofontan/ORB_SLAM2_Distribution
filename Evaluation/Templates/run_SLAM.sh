start=$SECONDS

#Inputs
system=${1?Error: no system} 				 # System: orbslam2 / orbslam3 / (idnav) / (dvoslam) / (badslam)
mode=${2?Error: no mode}				 # Mode: mono / (rgbd) / (stereo) / (visualInertial)
experimentName=${3?Error: no experimentName}	
experimentDate=${4?Error: no experimentDate} 	         # Date: yyyy_mm_dd
evaluateExperiment=${5?Error: no evaluateExperiment}   # if "y" TUM script evaluation is performed (conda---->python2)
activeVis=${6?Error: no activeVisualization}           # if "1" display visualization and terminal output
numRuns=${7?Error: no numberOfRuns}                    # number of executions per sequence
dataset=${8?Error: no dataset}			 # Dataset: kitti / rgbdtum / euroc / monotum / nuim / oxford / (madmax) / (fordva) / 
sequenceGroup=${9?Error: no sequenceGroup}		 # See file <getSequences.sh>
backwards=${10?Error: no backwards}			 # if "1" runs the sequence backwards
ws_path=${11?Error: no ws_path}	 	         # Workspace Path: "/home/alex" / "/home/fontan"
voc=${12?Error: no vocabulary}	 	         # Vocabulary: "superpoint_voc.yml" / "ORBvoc.txt"
sys_vs=${13?Error: no system_version}	 	         # System Version: ORB_SLAM2_Deterministic
resultsFolder=${14?Error: no resultsFolder}	 	 # Results Folder: ORB_SLAM2_results

echo ""
echo "Executing run_SLAM.sh ..."
echo "    system = $system"
echo "    mode = $mode"
echo "    experimentName = $experimentName"
echo "    experimentDate = $experimentDate"
echo "    evaluateExperiment = $evaluateExperiment"
echo "    activeVisualization = $activeVis"
echo "    numberOfRuns = $numRuns"
echo "    dataset = $dataset"
echo "    sequenceGroup = $sequenceGroup"
echo "    backwards = $backwards"
echo "    ws_path = $ws_path"
echo "    voc = $voc"
echo "    sys_vs = $sys_vs"
echo "    resultsFolder = $resultsFolder"

###################################################################################################
# Flags
#hpc='n'
#if [ ${ws_path} == "/home/fontan" ]
#then
#	hpc='y'
#fi
	
###################################################################################################	
# Path to scripts and folders
system_path="${ws_path}/${sys_vs}"
results_folder_dataset="${ws_path}/${resultsFolder}/${mode}/${dataset}"
executable="Examples/${mode}/${mode}_${dataset}"		
	
###################################################################################################	
# Path to dataset
. ./pathToDataset.sh

###################################################################################################
# Sequence configuration
. ./getSequences.sh

###################################################################################################
# Prepare experiment folder
if [ ! -d "$results_folder_dataset" ] 
then
	echo -e "\033[31mResults folder doesn't exist: ${results_folder_dataset}\033[0m"
	echo -e "Use script createResultsFolder.sh"	
	exit 0
fi

if [ $backwards == '1' ]
then
	experimentFolder="${results_folder_dataset}/${experimentDate}_${experimentName}_${sequenceGroup}_bwd"
else
	experimentFolder="${results_folder_dataset}/${experimentDate}_${experimentName}_${sequenceGroup}_fwd"
fi

echo "    Create experiment folder: ${experimentFolder} ..."
if [ ! -d "$experimentFolder" ] 
then
	mkdir $experimentFolder
	echo -e "        \033[32mCreated experiment folder.\033[0m"
else
	echo -e "       \033[31m Experiment folder already exists: ${experimentFolder} \033[0m"	
	echo -e "       \033[31m INCREASING EXPERIMENT IDs TO AVOID OVERWRITING !!!!!!!!!!!!!!!!! \033[0m"	
fi
###################################################################################################
echo -e "\n    Num. runs per sequence: $numRuns"
echo -e "\n    Sequences : "
for sequenceName in "${sequenceNames[@]}"
do
	sequenceFolder="$experimentFolder/$sequenceName"
	if [ ! -d "$sequenceFolder" ] 
	then
	        echo "        $sequenceName"   
		mkdir $sequenceFolder 	
	else
		echo -e "        $sequenceName : \033[31m Sequence folder already exists. INCREASING EXPERIMENT IDs. \033[0m" 
	fi    		
done

###################################################################################################
# Run system
if (( ${numRuns} > 0 ));
then
	if [ $system == 'orbslam2' ]
	then
		echo ""
		echo "    Running .${system_path}/${executable} ..."
	fi
fi
###################################################################################################
firstRun="0"
cd ${system_path}
for ((iRun = 0 ; iRun < $numRuns ; iRun++));
do
	seqIndex=0
	for sequenceName in "${sequenceNames[@]}"
	do
		sequenceFolder="$experimentFolder/$sequenceName"
		#sequence_stdout="$sequenceFolder/${system}_stdout.txt"
		sequencePath="${dataset_path}/${sequenceName}"
		
		previousRuns=($(ls ${sequenceFolder}/*system_output* 2>/dev/null)) 
		expId=${#previousRuns[@]}			
		echo "        in $sequenceName , exp id = $expId , with : ${sequenceSettings[seqIndex]}" 

		system_output="$sequenceFolder/system_output_${expId}"
		> $system_output
		
		if [ $system == 'orbslam2' ] || [ $system == 'orbslam3' ]
		then
			if [ $mode == 'mono' ]
			then
				. ./Evaluation/Templates/executables.sh
			fi
			
		fi

   		seqIndex=$((seqIndex + 1))
	done
done

###################################################################################################
#Evaluate results
if [ $evaluateExperiment == 'y' ]
then
	echo "    Evaluating trajectories... (activate conda) "
	tools_path="${system_path}/Evaluation/Templates"
	evaluate_ate_script="evaluate_ate.py";
	evaluate_rpe_script="evaluate_rpe.py";
	evaluate_ate_scale_script="evaluate_ate_scale_modified.py";
	evaluate_scale_script="evaluate_scale.py";
	
	cd ${system_path}/Evaluation/Templates
	#if [ ${hpc} == "n" ]
	#then
		source "${ws_path}/anaconda3/etc/profile.d/conda.sh"
		conda activate evalTUM
	#fi
	
	for sequenceName in "${sequenceNames[@]}"
	do	
		sequenceFolder="$experimentFolder/$sequenceName"
		evaluationSequenceFolder="$experimentFolder/$sequenceName/evaluationFiles"
		if [ ! -d "$evaluationSequenceFolder" ] 
		then
			mkdir $evaluationSequenceFolder 			
		fi    	
	
		. ./datasetConfiguration.sh

		if [ $mode == 'mono' ]
		then
			evaluationName="trajectory"			
			txtKeyFrameFiles=($(ls ${sequenceFolder}/*_KeyFrameTrajectory.txt))
			txtFrameFiles=($(ls ${sequenceFolder}/*_FrameTrajectory.txt))
			
			keyFrame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_KeyFrame_TUMformat_results.txt"
			frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results.txt"
			log_frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results_log.txt"			
			. ./evaluate_experiment.sh 
				
			evaluationName="before"
			txtKeyFrameFiles=($(ls ${sequenceFolder}/*_KeyFrameTrajectoryBeforeBA.txt))
			txtFrameFiles=($(ls ${sequenceFolder}/*_FrameTrajectoryBeforeBA.txt))
			
			keyFrame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_KeyFrame_TUMformat_results.txt"
			frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results.txt"
			log_frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results_log.txt"		
			. ./evaluate_experiment.sh 
			
			evaluationName="after"
			txtKeyFrameFiles=($(ls ${sequenceFolder}/*_KeyFrameTrajectoryAfterBA.txt))
			txtFrameFiles=($(ls ${sequenceFolder}/*_FrameTrajectoryAfterBA.txt))
			
			keyFrame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_KeyFrame_TUMformat_results.txt"
			frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results.txt"
			log_frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results_log.txt"		
			. ./evaluate_experiment.sh 
			
			evaluationName="probability"
			txtKeyFrameFiles=($(ls ${sequenceFolder}/*_${evaluationName}_AblationKeyFrame.txt))
			txtFrameFiles=($(ls ${sequenceFolder}/*_${evaluationName}_AblationFrame.txt))
			
			keyFrame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_KeyFrame_TUMformat_results.txt"
			frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results.txt"
			log_frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results_log.txt"		
			. ./evaluate_experiment.sh 
			
			evaluationName="chi2"
			txtKeyFrameFiles=($(ls ${sequenceFolder}/*_${evaluationName}_AblationKeyFrame.txt))
			txtFrameFiles=($(ls ${sequenceFolder}/*_${evaluationName}_AblationFrame.txt))
			
			keyFrame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_KeyFrame_TUMformat_results.txt"
			frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results.txt"
			log_frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results_log.txt"		
			. ./evaluate_experiment.sh 
			
			#evaluationName="exp"
			#txtKeyFrameFiles=($(ls ${sequenceFolder}/*_${evaluationName}_AblationKeyFrame.txt))
			#txtFrameFiles=($(ls ${sequenceFolder}/*_${evaluationName}_AblationFrame.txt))
			
			#keyFrame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_KeyFrame_TUMformat_results.txt"
			#frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results.txt"
			#log_frame_TUMformat_results="$evaluationSequenceFolder/${evaluationName}_Frame_TUMformat_results_log.txt"		
			#. ./evaluate_experiment.sh 
		fi
										
	done
	#if [ ${hpc} == "n" ]
	#then
		conda deactivate 
	#fi
	
else
	echo "No trajectory evaluation"
fi

duration=$(( SECONDS - start ))
experiment_duration="$experimentFolder/experimentDuration"
> $experiment_duration
(echo $duration) >> $experiment_duration	

