#include "Simulator.h"

using namespace cv;

int main(void) {
    bool arr[21][21]; // replace with obstacle_map
    Simulator sim; 
    sim.drawMap(arr);
    sim.interpretCoordinates();
    std::queue<Pather2::point> path;  // get path from Pather2 to pass in
    sim.drawPath(path);
    sim.drawRobot();
    imshow("Simulator", sim.img);
    int key = waitKey(0);
};

void Simulator::drawMap(bool obstacle_map[][21]) {
    
    for(int i; i < size; i++){
        for(int j; j < size; j++){
            if(obstacle_map[i][j]){
                  rectangle(img,
                    Point( w/21*i,w/21*j  ),
                    Point( w/21*(i+1), w/21*(j+1)),
                    Scalar( 255, 255, 255 ),
                    FILLED,
                    LINE_8 );
            }
        }
    }
};

void Simulator::drawPath(std::queue<Pather2::point, std::deque<Pather2::point, std::allocator<Pather2::point>>> path) {
    while(!path.empty()){ 
        Pather2::point start = path.front();
        path.pop();
        Pather2::point end = path.front();
        path.pop();

        cv::Point startcv{start.x * 21, start.y * 21}; // may be incorrect syntax
        cv::Point endcv{end.x * 21, end.y * 21};

        line(img,
        startcv,
        endcv,
        Scalar(0,0,0),
        2,
        LINE_8);
    }
};

void Simulator::interpretCoordinates() {

};

void Simulator::drawRobot() {
    int robotWidthHalf = 5;
    int robotHeightHalf = 10;
    rectangle(img,
        Point(w/2 - robotWidthHalf, w/2 - robotHeightHalf),
        Point(w/2 + robotWidthHalf, w/2 + robotHeightHalf),
        Scalar(255, 0, 0),
        FILLED,
        LINE_8);
};