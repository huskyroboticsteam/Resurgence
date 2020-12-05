git submodule update --init --recursive
mkdir ./build
cd build
cmake ../src
cd ..
cmake --build ./build
./build/tests
