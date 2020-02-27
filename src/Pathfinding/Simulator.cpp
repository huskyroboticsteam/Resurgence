#include Simulator.h

int main(void ){
    bool[21][21] arr; 
    drawMap(arr);
    interpretCoordinates();
    drawPath();
    drawRobot();
    imshow("Simulator", img);
    int key = waitKey(0);
};

void drawMap(bool obstacle_map[][]){

};

void drawPath(){

};

void interpretCoordinates(){

};

void drawRobot(){
    int robotWidthHalf = 5;
    int robotHeightHalf = 10;
    rectangle(img,
        Point(w/2 - robotWidthHalf, w/2 - robotHeightHalf),
        Point(w/2 + robotWidthHalf, w/2 + robotHeightHalf),
        Scalar(255, 0, 0),
        FILLED,
        LINE_8);
};