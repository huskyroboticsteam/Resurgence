//#include "ObstacleMap.h"
#include "Pather2.h"
#include <vector>
#include <queue>
#include <iostream>
#include <cmath>

//matrix size
#define ROW 10
#define COL 10


// print functions for debugging
void print_point(std::queue<Pather2::point> q) {
  while (!q.empty())
  {
    std::cout << "(" << q.front().x << ",";
    std::cout << q.front().y << ")";
    q.pop();
  }
  std::cout << std::endl;
}

void print_queue(std::queue<Pather2::queueNode> q) {
  while (!q.empty())
  {
    std::queue<Pather2::point> temp = q.front().path;
    print_point(temp);
    std::cout << std::endl;
    q.pop();
  }
  std::cout << std::endl;
}

//check if node is valid
bool isValid(int x, int y) {
    return ((x >= 0) && (x < ROW) && (y >= 0) && (y < COL));
}

int rowNum[] = {-1,0,0,1, -1, 1, 1, -1};
int rowCol[] = {0,-1,1,0, -1, -1, 1, 1};

Pather2::point BFS(bool map[][10], Pather2::point dest){

    Pather2::point src = {5,5};
    struct Pather2::point error = {-1, -1};
    //check if src and dest are represented with 1 not 0
    if ((map[src.x][src.y]) || (map[dest.x][dest.y])){
        //todo: change destination if destination is blocked
        return error;
    }

    map[src.x][src.y] = true;
    std::queue<Pather2::queueNode> q;
    std::queue<Pather2::point> newQueue;
    newQueue.push(src);
    Pather2::queueNode s = {src, 0, newQueue};
    q.push(s);

    // q is the queue of points that are waiting to be checked
    // curr.path is the path leading up to that point

    while (!q.empty()){
        //get front point in points to be checked and make it current
        Pather2::queueNode curr = q.front();
        Pather2::point pt = curr.pt;

        //check if current point is destination node
        if ((pt.x == dest.x) && (pt.y == dest.y)){
            //return first point after source in path to destination
            std::queue<Pather2::point> temp = curr.path;
            temp.pop();
            std::cout << "Final path: ";
            print_point(temp);
            return temp.front(); 
        }

        // pop current point off point to be checked
        q.pop();
 
        // iterate through adj points
        for (int i = 0; i < 8; i++){
            int row = pt.x + rowNum[i];
            int col = pt.y + rowCol[i];
            

            // check if adj point is not out of bounds and is not an obstacle
            if (isValid(row, col) && !map[row][col]){
                // set adj point as visited and push it to points to be checked(q)
                struct Pather2::point test = {row, col};
                std::queue<Pather2::point> addAdjPointToPath = curr.path;
                addAdjPointToPath.push(test);
                Pather2::queueNode adjNode = {{row, col}, curr.dist + 1, addAdjPointToPath};
                map[adjNode.pt.x][adjNode.pt.y] = true;
                q.push(adjNode);
            }
        }

    }
    return error;
}

Pather2::point getDest(Pather2::point GPSRobot, Pather2::point GPSDest) {

    const float PI = 3.1415927;
    struct Pather2::point dest;
    float ratioX = GPSDest.x - GPSRobot.x;
    float ratioY = GPSDest.y - GPSRobot.y;

    // check if dest directly horizontal or vertical
    if (ratioX == 0) {
        if (ratioY > 0) {
            dest.x = 10;
            dest.y = 0;
        } else {
            dest.x = 10;
            dest.y = 20;
        }
        return dest;
    } else if (ratioY == 0) {
        if (ratioX > 0) {
            dest.x = 20;
            dest.y = 10;
        } else {
            dest.x = 0;
            dest.y = 10;
        }
        return dest;
    }
    float ratio = ratioY / ratioX;
    float tanValue = atan(ratio) * 180 / PI;

    if (tanValue == 45) {
        dest.x = 0;
        dest.y = 20;
    } else if (tanValue == 135) {
        dest.x = 0;
        dest.y = 0;
    } else if (tanValue == 225) {
        dest.x = 0;
        dest.y = 20;
    }else if (tanValue == 315) {
        dest.x = 20;
        dest.y = 20;
    } else if (45 < tanValue && tanValue < 135) {
        int shiftInY = int((10 / ratioY) * ratioX);
        shiftInY += 10;
        dest.x = 0;
        dest.y = shiftInY;
    } else if (225 < tanValue && tanValue < 315) {
        int shiftInY = int((10 / ratioY) * ratioX);
        shiftInY += 10;
        dest.x = 20;
        dest.y = shiftInY;
    } else if ((tanValue > 315) || (tanValue < 45)){
        int shiftInX = int((10 / ratioX) * ratioY);
        shiftInX += 10;
        dest.x = shiftInX;
        dest.y = 20;
    } else if (135 < tanValue && tanValue < 225) {
        int shiftInX = int((10 / ratioX) * ratioY);
        shiftInX += 10;
        dest.x = shiftInX;
        dest.y = 0;
    }
    return dest;
}

// Pather2::Pather2(ObstacleMap obstacle_map)
// {
//     map = obstacle_map;   
// }

Pather2::point getPath(bool map[10][10], Pather2::point dest){
  Pather2::point rslt = BFS(map, dest);
  struct Pather2::point src = {4, 4};
  while(rslt.x == -1 && rslt.y == -1) {
    //do something
    //relocate the relative coordinate of the original target (some functions to determinate new end point relative to our local map)
    rslt = BFS(map, dest);
  }
  return rslt;
}


Pather2::point relocateDestination(Pather2::point dest){
  while(//new destination is not valide)
    if (dest.x == 20) {
      switch(dest.y) {
        case 0: dest.x -= 1;
                dest.y += 1;
                break;
        case 20: dest.x -= 1;
                dest.y -=;
                break;
        default: dest.x -= 1;
      }
    } else if (dest.x == 0) {
      switch(dest.y) {
        case 0: dest.x += 1;
                dest.y += 1;
                break;
        case 20: dest.y -= 1;
                dest.x += 1;
                break;
        default: dest.x += 1;
      }
    } else if (dest.y == 20) {
      dest.y -= 1;
    } else if (dest.y == 0) {
      dest.y += 1;
    } else if ()
    }
}

//If the relative location of the target is put into some invalid coordinate (unreachable),
//then assign any neighbor valid coordinate as the new relative location, and make search again. 

int main() {
    bool map[ROW][COL] = {{true, false, true, false, true, false, true, true, false, true},
                          {true, true, false, false, true, true, true, false, true, true},
                          {false, false, false, false, true, true, false, true, false, true},
                          {false, false, false, true, false, true, false, false, true, true},
                          {false, true, false, true, false, true, false, false, false, false},
                          {true, false, true, false, true, false, true, false, true, false},
                          {false, false, false, false, false, true, true, true, false, false},
                          {false, true, false, false, false, false, false, false, true, true},
                          {true, true, true, false, false, true, true, false, false, true}, 
                          {false, false, true, true, true, false, true, true, true, false}};
    struct Pather2::point destination = {9,9};
    struct Pather2::point nextPoint = getPath(map, destination);
    struct Pather2::point GPS_src = {0,0};
    struct Pather2::point GPS_dest = {70, 10};
    struct Pather2::point dest = getDest(GPS_src, GPS_dest);
    std::cout << "(" << dest.x << "," << dest.y << ")" << std::endl;
    std::cout << "First point in path: ";
    std::cout << "(" << nextPoint.x << "," << nextPoint.y << ")";
    std::cout << "" << std::endl;
    return 0;
}
