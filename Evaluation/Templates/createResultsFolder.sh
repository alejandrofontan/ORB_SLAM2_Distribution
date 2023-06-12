resultsFolder=${1?Error: no resultsFolder} 	

echo "Executing createResultsFolder.sh ..."

if [ ! -d "$resultsFolder" ] 
then
	echo -e "    \033[32mCreated results folder and subfolders: ${resultsFolder}\033[0m"
	mkdir $resultsFolder  
	. ./createResultsSubfolders.sh ${resultsFolder}
else
	echo -e "    \033[31mResults folder already exists: ${resultsFolder}\033[0m"
	echo -e "    \033[31mRISK OF OVERWRITING !!!!!!!! \033[0m"
	read -t 10 -p '    Do you want to OVERWRITE the results (yes/n): ' overwriteResults
	if [ ${overwriteResults} == 'yes' ] 
	then
		echo -e "    \033[31mOVERWRITING results folder and subfolders: ${resultsFolder}\033[0m"
		rm -rf ${resultsFolder}
		mkdir $resultsFolder  
		. ./createResultsSubfolders.sh ${resultsFolder}
	else
		echo "    Abort experiment."
		exit 0;
	fi
fi
