mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
echo "compile successfully"
./pathtracer -t 8 -s 2048 -a 64 0.05 -l 1 -m 5 -r 480 360 -f bunny.png ../dae/sky/CBbunny.dae
echo "run successfully"
