systemVersion=${1?Error: systemVersion}  
workspacePath=${2?Error: workspacePath} 

sleep 10
echo "Build ORB_SLAM2 interactive job: ${workspacePath}/${systemVersion} ..."
echo "Updating CMakeLists to hpc ..."

rm -rf "${workspacePath}/${systemVersion}/build/"
rm -rf "${workspacePath}/${systemVersion}/CMakeLists.txt"
rm -rf "${workspacePath}/${systemVersion}/Thirdparty/DBoW2/CMakeLists.txt"
rm -rf "${workspacePath}/${systemVersion}/Thirdparty/g2o/CMakeLists.txt"

cp "${workspacePath}/CMakeListsFolder/ORB_SLAM2/CMakeLists.txt" "${workspacePath}/${systemVersion}/CMakeLists.txt"
cp "${workspacePath}/CMakeListsFolder/DBoW2/CMakeLists.txt" "${workspacePath}/${systemVersion}/Thirdparty/DBoW2/CMakeLists.txt"
cp "${workspacePath}/CMakeListsFolder/g2o/CMakeLists.txt" "${workspacePath}/${systemVersion}/Thirdparty/g2o/CMakeLists.txt"

sleep 1

../Templates/build_ORB_SLAM2.sh ${systemVersion} ${workspacePath}
sleep 1

