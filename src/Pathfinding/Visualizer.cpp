#include "Visualizer.h"
#include "ObstacleMap.h"
#include <iostream>

using namespace cv;

void runTest(std::string description, ObstacleMap testMap, PointXY dest) {
    Visualizer sim;
    Pather2 pather;
    std::cout << "Text version of map:\n";
    testMap.print();
    sim.drawMap(testMap.obstacle_map);
    std::queue<PointXY> path = pather.BFS(dest);
    sim.drawPath(path);
    sim.drawRobot();
    sim.drawDestination();
    imshow("Visualizer test: " + description, sim.img);
    waitKey(0);
    destroyAllWindows();
}

int main(void) {
    // robot location for all tests: (0.0f, 0.0f)
    // Test1:
    ObstacleMap testMap1;
    std::vector<PointXY> points1 = {}; // supposed to be PointXY
    testMap1.update(points1, 0, 0);
    runTest("empty map", testMap1, PointXY{10.0f, 10.0f});

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
    testMap2.update(points2, 0, 0);
    runTest("diagonal obstacles", testMap2, PointXY{10.0f, 10.0f});

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
    testMap3.update(points3, 0, 0);
    runTest("irregular obstacles", testMap3, PointXY{10.0f, 10.0f});

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
    testMap4.update(points4, 0, 0);
    runTest("different goal", testMap4, PointXY{10.0f, 7.0f});

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
    testMap5.update(points5, 0, 0);
    runTest("another different goal", testMap5, PointXY{7.0f, 10.0f});

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
    testMap6.update(points6, 0, 0);
    runTest("more irregular obstacles", testMap6, PointXY{5.0f, 5.0f});
}

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
