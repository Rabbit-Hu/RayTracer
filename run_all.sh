if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir build
cd build
cmake ../src
make
mkdir ../output
# ./RayTracer ../input/spheres.txt ../output/spheres.png
# ./RayTracer ../input/cylinder.txt ../output/cylinder.png
# ./RayTracer ../input/paimon.txt ../output/paimon.png
# ./RayTracer ../input/slime.txt ../output/slime.png
# ./RayTracer ../input/prism.txt ../output/prism.png
# ./RayTracer ../input/rectangle.txt ../output/rectangle.png
./RayTracer ../input/wallpaper.txt ../output/wallpaper.png
cd ..