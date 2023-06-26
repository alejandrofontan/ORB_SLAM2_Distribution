#!/bin/bash -l 

#PBS -l ncpus=2
#PBS -l mem=30G 
#PBS -l walltime=18:00:00 
#PBS -l cputype=6140 

cd $PBS_O_WORKDIR
echo "Experiment SLAM Hpc: ${workspacePath}/${resultsFolder}"
echo "experimentName: ${experimentName}"
echo "systemVersion: ${systemVersion}"
echo "dataset: ${dataset}"
echo "sequenceGroup: ${sequenceGroup}"
echo "experimentDate: ${experimentDate}"
pwd

source /home/fontan/mambaforge/etc/profile.d/conda.sh
source /home/fontan/mambaforge/etc/profile.d/mamba.sh
mamba init
mamba activate orbslam2det

# Environment variables:
#
# experimentName
# resultsFolder       # Results Folder: ORB_SLAM2_results
# systemVersion       # System Version: ORB_SLAM2_Deterministic
# experimentDate      # Date: yyyy_mm_dd
# dataset             # Dataset: kitti / rgbdtum / euroc / monotum / (madmax) / (fordva) / 
# sequenceGroup       # See file <getSequences.sh>
# workspacePath       # Workspace Path: "/home/alex" / "/home/fontan"
# experimentComments

#Experiment settings
numberOfRuns="1"

evaluateExperiment="n"       # if "y" TUM script evaluation is performed (conda---->python2)
activeVisualization="0"      # if "1" display visualization and terminal output

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


#fileCpuEsp="${workspacePath}/${resultsFolder}/cpuEspecifications.txt"
#> ${fileCpuEsp}
#lscpu >> ${fileCpuEsp}

#experimentSettingsFile="${workspacePath}/${resultsFolder}/experimentSettings.txt"
#> ${experimentSettingsFile}
#(echo "Experiment name: $experimentName") >> ${experimentSettingsFile}   	
#(echo "Experiment date: $experimentDate") >> ${experimentSettingsFile}   	
#(echo "System version: $systemVersion") >> ${experimentSettingsFile}   	
#(echo "Dataset: $dataset") >> ${experimentSettingsFile}   	
#(echo "Sequence Group: $sequenceGroup") >> ${experimentSettingsFile}   	
#(echo "Experiment Comments: ${experimentComments}") >> ${experimentSettingsFile}   	
#(echo "Vocabulary File: ${vocabularyFile}") >> ${experimentSettingsFile}   	


./fwd_bwd_runs_hpc.sh ${system} ${mode} ${experimentName} ${experimentDate} ${evaluateExperiment} ${activeVisualization} ${numberOfRuns} ${dataset} ${sequenceGroup} ${workspacePath} ${vocabularyFile} ${systemVersion} ${resultsFolder}

