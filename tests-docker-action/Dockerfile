FROM spmallick/opencv-docker:opencv
COPY . .
CMD git clone https://github.com/catchorg/Catch2.git
CMD cd Catch2
CMD cmake -Bbuild -H. -DBUILD_TESTING=OFF
CMD sudo cmake --build build/ --target install
CMD cd ..
CMD mkdir build
CMD cmake -Bbuild ./src
CMD cmake --build build
