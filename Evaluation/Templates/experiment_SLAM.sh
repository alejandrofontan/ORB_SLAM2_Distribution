echo "Executing experiment_SLAM.sh ..."

#Input parameters
experimentName=${1?Error: experimentName}  
resultsFolder=${2?Error: resultsFolder}   # Results Folder: ORB_SLAM2_results

systemVersion=${3?Error: systemVersion}   # System Version: ORB_SLAM2_Deterministic
experimentDate=${4?Error: experimentDate} # Date: yyyy_mm_dd

dataset=${5?Error: dataset}               # Dataset: kitti / rgbdtum / euroc / monotum / (madmax) / (fordva) / 
sequenceGroup=${6?Error: sequenceGroup}   # See file <getSequences.sh>

workspacePath=${7?Error: workspacePath}   # Workspace Path: "/home/alex" / "/home/fontan"

experimentComments=${8?Error: experimentComments} 

#Experiment settings
numberOfRuns="2"

evaluateExperiment="y"       # if "y" TUM script evaluation is performed (conda---->python2)
activeVisualization="1"      # if "1" display visualization and terminal output

# System configuration
system="orbslam2"            # System: orbslam2 / orbslam3 / (idnav) / (dvoslam) / (badslam)
mode="mono"                  # Mode: mono / (rgbd) / (stereo) / (visualInertial)

vocabularyFile="ORBvoc.txt"  # "superpoint_voc.yml" / "ORBvoc.txt"

if [ ! -d "${workspacePath}/${resultsFolder}" ] 
then
	echo -e "    \033[31mResults folder doesn't exist: ${workspacePath}/${resultsFolder}\033[0m"
	echo -e "    Use script createResultsFolder.sh"	
	exit 0
fi

fileCpuEsp="${workspacePath}/${resultsFolder}/cpuEspecifications.txt"
> ${fileCpuEsp}
lscpu >> ${fileCpuEsp}

experimentSettingsFile="${workspacePath}/${resultsFolder}/experimentSettings.txt"
> ${experimentSettingsFile}
(echo "Experiment name: $experimentName") >> ${experimentSettingsFile}   	
(echo "Experiment date: $experimentDate") >> ${experimentSettingsFile}   	
(echo "System version: $systemVersion") >> ${experimentSettingsFile}   	
(echo "Dataset: $dataset") >> ${experimentSettingsFile}   	
(echo "Sequence Group: $sequenceGroup") >> ${experimentSettingsFile}   	
(echo "Experiment Comments: ${experimentComments}") >> ${experimentSettingsFile}   	
(echo "Vocabulary File: ${vocabularyFile}") >> ${experimentSettingsFile}   	

backwards="0"                # if "1" runs the sequence backwards
./run_SLAM.sh ${system} ${mode} ${experimentName} ${experimentDate} ${evaluateExperiment} ${activeVisualization} ${numberOfRuns} ${dataset} ${sequenceGroup} ${backwards} ${workspacePath} ${vocabularyFile} ${systemVersion} ${resultsFolder}

backwards="1"                # if "1" runs the sequence backwards
./run_SLAM.sh ${system} ${mode} ${experimentName} ${experimentDate} ${evaluateExperiment} ${activeVisualization} ${numberOfRuns} ${dataset} ${sequenceGroup} ${backwards} ${workspacePath} ${vocabularyFile} ${systemVersion} ${resultsFolder}
