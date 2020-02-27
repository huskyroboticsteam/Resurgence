#include "Simulator.h"

int main(void ){
    
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