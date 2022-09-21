mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
echo "compile successfully"
./meshedit ../dae/beetle.dae 
echo "run successfully"
