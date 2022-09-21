mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
echo "compile successfully"
./csc4140_a1_p2_1 >> ./p2_1.out
./csc4140_a1_p2_2 >> ./p2_2.out
./csc4140_a1_p2_3 ../lenna.png >> ./p2_3.out
./csc4140_a1_p2_4 >> ./p2_4.out
echo "run successfully"
