echo "test"
git clone https://github.com/andrewbriand/urg_library-1.2.5.git
cd urg_library-1.2.5
make
make install
cd ..
git clone https://github.com/catchorg/Catch2.git
cd ./Catch2
cmake -B./build -H. -DBUILD_TESTING=OFF
cmake --build ./build/ --target install
cd ..
mkdir ./build
cd build
cmake ../src
cd ..
cmake --build ./build
./build/tests
