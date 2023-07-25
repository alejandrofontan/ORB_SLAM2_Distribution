# Input parameters
systemVersion=${1?Error: systemVersion}  
workspacePath=${2?Error: workspacePath}  
systemPath="${workspacePath}/${systemVersion}"

echo ""
echo "Executing build_ORB_SLAM2.sh ..."
echo "    systemVersion = $systemVersion"
echo "    workspacePath = $workspacePath"
echo ""

cd "$systemPath"

echo "    Configuring and building Thirdparty/DBoW2 (output hidden) ..."

cd Thirdparty/DBoW2
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
make -j12 > /dev/null 2>&1
make -j1 | sed 's/^/        /'

cd ../../g2o

echo "    Configuring and building Thirdparty/g2o (output hidden) ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
make -j12 > /dev/null 2>&1
make -j1 | sed 's/^/        /'

cd ../../../

echo "    Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "    Configuring and building ORB_SLAM2 (output hidden) ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
make -j12 > /dev/null 2>&1
make -j1 | sed 's/^/        /'
