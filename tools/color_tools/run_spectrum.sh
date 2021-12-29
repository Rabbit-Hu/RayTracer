if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir build
cd build
cmake ..
make
mkdir ../output
./spectrum
cd ..