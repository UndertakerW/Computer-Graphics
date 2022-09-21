mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
echo "compile successfully"
./pathtracer -t 6 -s 256 -l 16 -m 5 -r 480 360 -a 64 0.05 -b 0.05 -d 2 -c CBdragon_cam_settings.txt -f CBdragon_256_16_5_0.05_2.png ../dae/sky/CBdragon.dae
echo "run successfully"
