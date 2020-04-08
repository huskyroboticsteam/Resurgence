#include "Simulator.h"
#include "ObstacleMap.h"

using namespace cv;

int main(void) {
    // robot location for all tests: (0.0f, 0.0f)
    // Test1: destination = (10.0f, 10.0f)
    ObstacleMap testMap1;
    std::vector<Point> points1 = {}; // supposed to be Point.h
    testMap1.update(points1);
    runTest(testMap1);

    // Test2: destination = (10.0f, 10.0f)
    ObstacleMap testMap2;
    std::vector<Point> points2 = {
        Point{1.0f, 1.0f},
        Point{2.0f, 2.0f},
        Point{3.0f, 3.0f},
        Point{4.0f, 4.0f},
        Point{5.0f, 5.0f},
        Point{6.0f, 6.0f},
        Point{7.0f, 7.0f},
        Point{8.0f, 8.0f},
        Point{9.0f, 9.0f},
        Point{10.0f, 10.0f}
    };
    testMap2.update(points2);
    runTest(testMap2);

    // Test3: destination = (10.0f, 10.0f)
    ObstacleMap testMap3;
    std::vector<Point> points3 = {
        Point{2.0f, 2.0f},
        Point{-2.0f, -2.0f},
        Point{-3.0f, 3.0f},
        Point{3.0f, -3.0f},
        Point{5.0f, -5.0f},
        Point{1.0f, 1.0f}
    };
    testMap3.update(points3);
    runTest(testMap3);

    // Test4: destination = (10.0f, 7.0f)
    ObstacleMap testMap4;
    std::vector<Point> points4 = {
        Point{1.0f, 1.0f},
        Point{2.0f, 2.0f},
        Point{3.0f, 3.0f},
        Point{4.0f, 4.0f},
        Point{5.0f, 5.0f},
        Point{6.0f, 6.0f},
        Point{7.0f, 7.0f},
        Point{8.0f, 8.0f},
        Point{9.0f, 9.0f},
        Point{10.0f, 10.0f}
    };
    testMap3.update(points4);
    runTest(testMap4);

    // Test5: destination = (7.0f, 10.0f)
    ObstacleMap testMap5;
    std::vector<Point> points5 = {
        Point{1.0f, 1.0f},
        Point{2.0f, 2.0f},
        Point{3.0f, 3.0f},
        Point{4.0f, 4.0f},
        Point{5.0f, 5.0f},
        Point{6.0f, 6.0f},
        Point{7.0f, 7.0f},
        Point{8.0f, 8.0f},
        Point{9.0f, 9.0f},
        Point{10.0f, 10.0f}
    };
    testMap3.update(points5);
    runTest(testMap5);

    // Test6: destination = (5.0f, 5.0f)
    ObstacleMap testMap6;
    std::vector<Point> points6 = {
        Point{1.0f, 1.0f},
        Point{2.0f, 2.0f},
        Point{3.0f, 3.0f},
        Point{4.0f, 4.0f},
        Point{5.0f, 5.0f},
        Point{2.0f, 4.0f},
        Point{3.0f, 7.0f},
        Point{5.0f, 2.0f},
        Point{6.0f, 4.0f},
        Point{1.0f, 3.0f}
    };
    testMap3.update(points6);
    runTest(testMap6);

};

void runTest(ObstacleMap testMap) {
    Simulator sim; 
    sim.drawMap(testMap.obstacle_map);
    sim.interpretCoordinates();
    std::queue<Pather2::point> path;  // get path from Pather2 to pass in, change to use point.h
    sim.drawPath(path);
    sim.drawRobot();
    sim.drawDestination();
    imshow("Simulator", sim.img);
    int key = waitKey(0);

}

void Simulator::drawMap(bool obstacle_map[][21]) {
    
    for(int i; i < size; i++){
        for(int j; j < size; j++){
            if(obstacle_map[i][j]){
                  rectangle(img,
                    cv::Point(w/21*i, w/21*j ),
                    cv::Point(w/21*(i+1), w/21*(j+1)),
                    Scalar(255, 255, 255),
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
        cv::Point(w/2 - robotWidthHalf, w/2 - robotHeightHalf),
        cv::Point(w/2 + robotWidthHalf, w/2 + robotHeightHalf),
        Scalar(255, 0, 0),
        FILLED,
        LINE_8);
};

void Simulator::drawDestination() {
        // mark edge location closest to destination or draw destination if in bounds
        // get destination info from pather2
};