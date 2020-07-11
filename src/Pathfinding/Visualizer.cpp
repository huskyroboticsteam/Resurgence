#include "Visualizer.h"
#include "ObstacleMap.h"

using namespace cv;

int main(void) {
    // robot location for all tests: (0.0f, 0.0f)
    // Test1:
    ObstacleMap testMap1;
    std::vector<PointXY> points1 = {}; // supposed to be PointXY
    testMap1.update(points1);
    runTest(testMap1, PointXY{10.0f, 10.0f});

    // Test2:
    ObstacleMap testMap2;
    std::vector<PointXY> points2 = {
        PointXY{1.0f, 1.0f},
        PointXY{2.0f, 2.0f},
        PointXY{3.0f, 3.0f},
        PointXY{4.0f, 4.0f},
        PointXY{5.0f, 5.0f},
        PointXY{6.0f, 6.0f},
        PointXY{7.0f, 7.0f},
        PointXY{8.0f, 8.0f},
        PointXY{9.0f, 9.0f},
        PointXY{10.0f, 10.0f}
    };
    testMap2.update(points2);
    runTest(testMap2, PointXY{10.0f, 10.0f});

    // Test3:
    ObstacleMap testMap3;
    std::vector<PointXY> points3 = {
        PointXY{2.0f, 2.0f},
        PointXY{-2.0f, -2.0f},
        PointXY{-3.0f, 3.0f},
        PointXY{3.0f, -3.0f},
        PointXY{5.0f, -5.0f},
        PointXY{1.0f, 1.0f}
    };
    testMap3.update(points3);
    runTest(testMap3, PointXY{10.0f, 10.0f});

    // Test4:
    ObstacleMap testMap4;
    std::vector<PointXY> points4 = {
        PointXY{1.0f, 1.0f},
        PointXY{2.0f, 2.0f},
        PointXY{3.0f, 3.0f},
        PointXY{4.0f, 4.0f},
        PointXY{5.0f, 5.0f},
        PointXY{6.0f, 6.0f},
        PointXY{7.0f, 7.0f},
        PointXY{8.0f, 8.0f},
        PointXY{9.0f, 9.0f},
        PointXY{10.0f, 10.0f}
    };
    testMap3.update(points4);
    runTest(testMap4, PointXY{10.0f, 7.0f});

    // Test5:
    ObstacleMap testMap5;
    std::vector<PointXY> points5 = {
        PointXY{1.0f, 1.0f},
        PointXY{2.0f, 2.0f},
        PointXY{3.0f, 3.0f},
        PointXY{4.0f, 4.0f},
        PointXY{5.0f, 5.0f},
        PointXY{6.0f, 6.0f},
        PointXY{7.0f, 7.0f},
        PointXY{8.0f, 8.0f},
        PointXY{9.0f, 9.0f},
        PointXY{10.0f, 10.0f}
    };
    testMap3.update(points5);
    runTest(testMap5, PointXY{7.0f, 10.0f});

    // Test6:
    ObstacleMap testMap6;
    std::vector<PointXY> points6 = {
        PointXY{1.0f, 1.0f},
        PointXY{2.0f, 2.0f},
        PointXY{3.0f, 3.0f},
        PointXY{4.0f, 4.0f},
        PointXY{5.0f, 5.0f},
        PointXY{2.0f, 4.0f},
        PointXY{3.0f, 7.0f},
        PointXY{5.0f, 2.0f},
        PointXY{6.0f, 4.0f},
        PointXY{1.0f, 3.0f}
    };
    testMap3.update(points6);
    runTest(testMap6, PointXY{5.0f, 5.0f});

};

void runTest(ObstacleMap testMap, PointXY dest) {
    Visualizer sim; 
    Pather2 pather;
    sim.drawMap(testMap.obstacle_map);
    sim.interpretCoordinates();
    std::queue<PointXY> path = pather.BFS(testMap.obstacle_map, dest);
    sim.drawPath(path);
    sim.drawRobot();
    sim.drawDestination();
    imshow("Visualizer", sim.img);
    int key = waitKey(0);

}

void Visualizer::drawMap(bool obstacle_map[][21]) {
    
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

void Visualizer::drawPath(std::queue<PointXY, std::deque<PointXY, std::allocator<PointXY>>> path) {
    while(!path.empty()){ 
        PointXY start = path.front();
        path.pop();
        PointXY end = path.front();
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

void Visualizer::interpretCoordinates() {

};

void Visualizer::drawRobot() {
    int robotWidthHalf = 5;
    int robotHeightHalf = 10;
    rectangle(img,
        cv::Point(w/2 - robotWidthHalf, w/2 - robotHeightHalf),
        cv::Point(w/2 + robotWidthHalf, w/2 + robotHeightHalf),
        Scalar(255, 0, 0),
        FILLED,
        LINE_8);
};

void Visualizer::drawDestination() {
        // mark edge location closest to destination or draw destination if in bounds
        // get destination info from pather2
};