#Inputs							
echo ""
echo "Executing evaluate_experiment.sh : ${evaluationName}"

keyFrameTrajectory_TUMformat_results="${sequenceFolder}/${evaluationName}_KeyFrame_TUMformat_results.txt"
frameTrajectory_TUMformat_results="${sequenceFolder}/${evaluationName}_Frame_TUMformat_results.txt"
> $keyFrameTrajectory_TUMformat_results
> $frameTrajectory_TUMformat_results

echo "    evaluating ${#txtKeyFrameFiles[@]} KeyFrame trajectories of $sequenceName"
echo "    evaluating ${#txtFrameFiles[@]} Frame trajectories of $sequenceName"
		
txtIndex=0
for j in "${txtKeyFrameFiles[@]}"
do
	echo "        $j"
	trajectoryFile="${j%.txt}"
	alignedTrajectoryFile="${trajectoryFile}_aligned"
	> $alignedTrajectoryFile
	alignedTrajectoryPlot="${trajectoryFile}_alignedPlot"
	
	numKey_ate_scale=$(python2 ${evaluate_ate_scale_script} --plot ${alignedTrajectoryPlot} --save_associations ${alignedTrajectoryFile} --max_difference ${max_diff} ${groundtruth_file} $j)
	scale=$(python2 ${evaluate_scale_script} --max_difference ${max_diff} $groundtruth_file $j)					
				
	rpe=$(python2 ${evaluate_rpe_script} --delta ${frequence}  --delta_unit 'f' --scale ${scale} $groundtruth_file $j)
	numKey_ate_scale_rpe="${numKey_ate_scale} ${rpe}"
	(echo ${numKey_ate_scale_rpe}) >> $keyFrameTrajectory_TUMformat_results	
	txtIndex=$((txtIndex + 1))	    	      	          
done
	
txtIndex=0
for j in "${txtFrameFiles[@]}"
do
	echo "        $j"
	trajectoryFile="${j%.txt}"
	alignedTrajectoryFile="${trajectoryFile}_aligned"
	> $alignedTrajectoryFile
	alignedTrajectoryPlot="${trajectoryFile}_alignedPlot"
	
	numKey_ate_scale=$(python2 ${evaluate_ate_scale_script} --plot ${alignedTrajectoryPlot} --save_associations ${alignedTrajectoryFile} --max_difference ${max_diff} ${groundtruth_file} $j)
	scale=$(python2 ${evaluate_scale_script} --max_difference ${max_diff} $groundtruth_file $j)					
				
	rpe=$(python2 ${evaluate_rpe_script} --delta ${frequence}  --delta_unit 'f' --scale ${scale} $groundtruth_file $j)
	numKey_ate_scale_rpe="${numKey_ate_scale} ${rpe}"
	(echo ${numKey_ate_scale_rpe}) >> $frameTrajectory_TUMformat_results	
	
	(echo ${trajectoryFile}) >> $log_frame_TUMformat_results	
	txtIndex=$((txtIndex + 1))	    	      	          
done		
			
