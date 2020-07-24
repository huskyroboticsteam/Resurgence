#include "Visualizer.h"
#include "ObstacleMap.h"

using namespace cv;

void Visualizer::drawMap(bool obstacle_map[][GRID_SIZE]) {
    for(int i=0; i < GRID_SIZE; i++){
        for(int j=0; j < GRID_SIZE; j++){
            if(obstacle_map[i][j]){
                rectangle(img,
                  cv::Point2d(w/GRID_SIZE*i, w/GRID_SIZE*j ),
                  cv::Point2d(w/GRID_SIZE*(i+1), w/GRID_SIZE*(j+1)),
                  Scalar(255, 255, 255),
                  FILLED,
                  LINE_8 );
            }
        }
    }
}

void Visualizer::drawPath(std::queue<PointXY, std::deque<PointXY, std::allocator<PointXY>>> path) {
    while(!path.empty()){
        PointXY start = path.front();
        path.pop();
        if (path.empty()) break;
        PointXY end = path.front();
        double scale = w/GRID_SIZE;

        cv::Point2d startcv{start.x * scale, start.y * scale};
        cv::Point2d endcv{end.x * scale, end.y * scale};

        line(img,
        startcv,
        endcv,
        Scalar(0,0,255),
        2,
        LINE_8);
    }
}

void Visualizer::drawRobot() {
    int robotWidthHalf = 5;
    int robotHeightHalf = 10;
    rectangle(img,
        cv::Point2d(w/2 - robotWidthHalf, w/2 - robotHeightHalf),
        cv::Point2d(w/2 + robotWidthHalf, w/2 + robotHeightHalf),
        Scalar(255, 0, 0),
        FILLED,
        LINE_8);
}

void Visualizer::drawDestination() {
        // mark edge location closest to destination or draw destination if in bounds
        // get destination info from pather2
}
