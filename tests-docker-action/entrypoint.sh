echo "test"
apt-get install -y mercurial
hg clone http://hg.code.sf.net/p/urgnetwork/urg_library 
cd urg_library
make
make install
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
