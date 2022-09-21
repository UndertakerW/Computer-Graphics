mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
echo "compile successfully"
./pathtracer -t 6 -s 16 -l 16 -m 5 -w 5 -r 480 360 ../dae/sky/CBspheres.dae -f CBspheres_16_16_5_5.png
echo "run successfully"
