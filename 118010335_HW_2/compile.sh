mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
echo "compile successfully"
cp ../parameters.xml ./parameters.xml
echo "copy xml file successfully"
./main
echo "run successfully"
