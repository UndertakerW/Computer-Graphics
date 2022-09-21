mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
echo "compile successfully"
./draw ../svg/basic/test4.svg 
echo "run successfully"
