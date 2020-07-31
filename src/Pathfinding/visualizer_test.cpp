#include "Visualizer.h"
#include "ObstacleMap.h"
#include <iostream>

using namespace cv;

void runTest(std::string description, const std::vector<PointXY>& obstacles, PointXY dest) {
    Visualizer sim;
    Pather2 pather;
    pather.updateMap(obstacles, 0.0, 0.0);
    std::cout << "Text version of map:\n";
    pather.obsMap.print();
    sim.drawMap(pather.obsMap.obstacle_map);
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
    std::vector<PointXY> points1 = {}; // supposed to be PointXY
    runTest("empty map", points1, PointXY{10.0f, 10.0f});

    // Test2:
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
    runTest("diagonal obstacles", points2, PointXY{10.0f, 10.0f});

    // Test3:
    std::vector<PointXY> points3 = {
        PointXY{2.0f, 2.0f},
        PointXY{-2.0f, -2.0f},
        PointXY{-3.0f, 3.0f},
        PointXY{3.0f, -3.0f},
        PointXY{5.0f, -5.0f},
        PointXY{1.0f, 1.0f}
    };
    runTest("irregular obstacles", points3, PointXY{10.0f, 10.0f});

    // Test4:
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
    runTest("different goal", points4, PointXY{10.0f, 7.0f});

    // Test5:
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
    runTest("another different goal", points5, PointXY{7.0f, 10.0f});

    // Test6:
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
    runTest("more irregular obstacles", points6, PointXY{5.0f, 5.0f});
}

