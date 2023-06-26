
#Inputs
system=${1?Error: no system} 				 # System: orbslam2 / orbslam3 / (idnav) / (dvoslam) / (badslam)
mode=${2?Error: no mode}				 # Mode: mono / (rgbd) / (stereo) / (visualInertial)
experimentName=${3?Error: no experimentName}	
experimentDate=${4?Error: no experimentDate} 	         # Date: yyyy_mm_dd
evaluateExperiment=${5?Error: no evaluateExperiment}   # if "y" TUM script evaluation is performed (conda---->python2)
activeVisualization=${6?Error: no activeVisualization} # if "1" display visualization and terminal output
numberOfRuns=${7?Error: no numberOfRuns}               # number of executions per sequence
dataset=${8?Error: no dataset}			 # Dataset: kitti / rgbdtum / euroc / monotum / (madmax) / (fordva) / 
sequenceGroup=${9?Error: no sequenceGroup}		 # See file <getSequences.sh>
workspacePath=${10?Error: no ws_path}	 	         # Workspace Path: "/home/alex" / "/home/fontan"
vocabularyFile=${11?Error: no vocabulary}	 	 # Vocabulary: "superpoint_voc.yml" / "ORBvoc.txt"
systemVersion=${12?Error: no system_version}	 	 # System Version: ORB_SLAM2_Deterministic
resultsFolder=${13?Error: no resultsFolder}	 	 # Results Folder: ORB_SLAM2_results

echo "Fwd Bwd Runs HPC: ${workspacePath}/${resultsFolder}"

cd ../Templates
backwards="0"                # if "1" runs the sequence backwards
./run_SLAM.sh ${system} ${mode} ${experimentName} ${experimentDate} ${evaluateExperiment} ${activeVisualization} ${numberOfRuns} ${dataset} ${sequenceGroup} ${backwards} ${workspacePath} ${vocabularyFile} ${systemVersion} ${resultsFolder}

#backwards="1"                # if "1" runs the sequence backwards
#./run_SLAM.sh ${system} ${mode} ${experimentName} ${experimentDate} ${evaluateExperiment} ${activeVisualization} ${numberOfRuns} ${dataset} ${sequenceGroup} ${backwards} ${workspacePath} ${vocabularyFile} ${systemVersion} ${resultsFolder}
cd ../Executables
