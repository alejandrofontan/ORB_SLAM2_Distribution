# Input parameters
systemVersion=${1?Error: systemVersion}  
workspacePath=${2?Error: workspacePath}  

systemPath="${workspacePath}/${systemVersion}"

echo ""
echo "Executing build_ORB_SLAM2_fast.sh ..."
echo "    systemVersion = $systemVersion"
echo "    workspacePath = $workspacePath"
echo ""

cd "$systemPath"

echo "Configuring and building ORB_SLAM2 ..."

cd build
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
make -j16 > /dev/null 2>&1
make -j1 | sed 's/^/        /'
