git clone https://github.com/andrewbriand/urg_library-1.2.5.git
cd urg_library-1.2.5
make
make install
cd ..
apt-get update
apt-get install -y libsfml-dev
# This next line is apparently where Eigen is installed
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:`pwd`src/mapping/EKFSlam
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/usr/include/eigen3
git submodule update --init --recursive
git clone https://github.com/catchorg/Catch2.git
cd ./Catch2
git checkout v2.13.2
cmake -B./build -H. -DBUILD_TESTING=OFF
cmake --build ./build/ --target install
cd ..
mkdir ./build
cd build
cmake ../src
cd ..
cmake --build ./build
./build/tests
