#include "Simulator.h"

int main(void ){
    bool[21][21] arr; 
    drawMap(arr);
    interpretCoordinates();
    drawPath();
    drawRobot();
    imshow("Simulator", img);
    int key = waitKey(0);
};

void Simulator::drawMap(bool obstacle_map[][21]){
    
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

void Simulator::drawPath(){

};

void Simulator::interpretCoordinates(){

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