#include "Pather2.h"
#include <vector>
#include <queue>
#include <iostream>
#include <cmath>
#include "Point.h"
 
//matrix size
// #define ROW 10
// #define COL 10
 
// print functions for debugging
void print_point(std::queue<Point> q) {
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
       std::queue<Point> temp = q.front().path;
       print_point(temp);
       std::cout << std::endl;
       q.pop();
   }
   std::cout << std::endl;
}
 
// returns full path
std::queue<Point> BFS(bool map[][21], Point dest){
  
   int rowNum[] = {-1,0,0,1, -1, 1, 1, -1};
   int rowCol[] = {0,-1,1,0, -1, -1, 1, 1};
  
   Point src = {5,5};
   std::queue<Point> error;
   error.push(src);
   //check if src and dest are represented with 1 not 0
   if ((map[src.x][src.y]) || (map[dest.x][dest.y])){
       //todo: change destination if destination is blocked
       return error;
   }
  
   map[src.x][src.y] = true;
   std::queue<Pather2::queueNode> q;
   std::queue<Point> newQueue;
   newQueue.push(src);
   Pather2::queueNode s = {src, 0, newQueue};
   q.push(s);
  
   // q is the queue of points that are waiting to be checked
   // curr.path is the path leading up to that point
  
   while (!q.empty()){
       //get front point in points to be checked and make it current
       Pather2::queueNode curr = q.front();
       Point pt = curr.pt;
      
       //check if current point is destination node
       if ((pt.x == dest.x) && (pt.y == dest.y)){
           //return first point after source in path to destination
           std::queue<Point> temp = curr.path;
           temp.pop();
           std::cout << "Final path: ";
           print_point(temp);
          
           return temp;
       }
      
       // pop current point off point to be checked
       q.pop();
      
       // iterate through adj points
       for (int i = 0; i < 8; i++){
           int row = pt.x + rowNum[i];
           int col = pt.y + rowCol[i];
          
           //((row >= 0) && (row < 21) && (col >= 0) && (col < 21))
           // check if adj point is not out of bounds and is not an obstacle
           if (((row >= 0) && (row < 21) && (col >= 0) && (col < 21)) && !map[row][col]){
               // set adj point as visited and push it to points to be checked(q)
               struct Point test = {row, col};
               std::queue<Point> addAdjPointToPath = curr.path;
               addAdjPointToPath.push(test);
               Pather2::queueNode adjNode = {{row, col}, curr.dist + 1, addAdjPointToPath};
               map[adjNode.pt.x][adjNode.pt.y] = true;
               q.push(adjNode);
           }
       }
      
   }
   return error;
}
 
// helper BFS method to return first point in path
Point mainBFS(bool map[][21], Point dest) {
   std::queue<Point> path = BFS(map, dest);
   return path.front();
  
}
 
Point getDest(Point GPSRobot, Point GPSDest) {
  
   const float PI = atan(1) * 4;
   struct Point dest;
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
 
 
//If the relative location of the target is put into some invalid coordinate (unreachable),
//then assign any neighbor valid coordinate as the new relative location, and make search again.
Point relocateDestination(Point dest, int shrink_constant){
   if (dest.x == 20 - shrink_constant) {
       switch(dest.y) {
           case 0: dest.x -= 1;
           dest.y += 1;
           break;
           case 20: dest.x -= 1;
           dest.y -= 1;
           break;
           default: dest.x -= 1;
       }
   } else if (dest.x == 0 + shrink_constant) {
       switch(dest.y) {
           case 0: dest.x += 1;
           dest.y += 1;
           break;
           case 20: dest.y -= 1;
           dest.x += 1;
           break;
           default: dest.x += 1;
       }
   } else if (dest.y == 20 - shrink_constant) {
       dest.y -= 1;
   } else if (dest.y == 0 + shrink_constant) {
       dest.y += 1;
   }
   return dest;
}
 
 
Point getPath(bool map[][21], Point dest){
 Point rslt = mainBFS(map, dest);
 struct Point src = {4, 4};
 int shrink_constant = 0;
 while(rslt.x == -1 && rslt.y == -1) {
   dest = relocateDestination(dest, shrink_constant);
   //do something
   //relocate the relative coordinate of the original target (some functions to determinate new end point relative to our local map)
   rslt = mainBFS(map, dest);
   shrink_constant += 1;
 }
 return rslt;
}
 
int returnHeading(bool map[][21]) {
       struct Point GPS_src = {0,0};
       struct Point GPS_dest = {70, 10};
       struct Point destination = getDest(GPS_src, GPS_dest);
       struct Point nextPoint = getPath(map, destination);
       std::cout << "(" << destination.x << "," << destination.y << ")" << std::endl;
       std::cout << "First point in path: ";
       std::cout << "(" << nextPoint.x << "," << nextPoint.y << ")";
       std::cout << "" << std::endl;
      
       int xDiff = nextPoint.y - 10;
       int yDiff = nextPoint.x - 10;
       const float PI = atan(1) * 4;
       int ratio = yDiff / xDiff;
       float headingAngle = atan(ratio) * 180 / PI;
       headingAngle -= 90;
       return headingAngle;
}
 
int main() {
   // bool map[ROW][COL] = {{true, false, true, false, true, false, true, true, false, true},
   //                       {true, true, false, false, true, true, true, false, true, true},
   //                       {false, false, false, false, true, true, false, true, false, true},
   //                       {false, false, false, true, false, true, false, false, true, true},
   //                       {false, true, false, true, false, true, false, false, false, false},
   //                       {true, false, true, false, true, false, true, false, true, false},
   //                       {false, false, false, false, false, true, true, true, false, false},
   //                       {false, true, false, false, false, false, false, false, true, true},
   //                       {true, true, true, false, false, true, true, false, false, true},
   //                       {false, false, true, true, true, false, true, true, true, false}};
   // struct Pather2::point destination = {9,9};
   // struct Pather2::point nextPoint = getPath(map, destination);
   // struct Pather2::point GPS_src = {0,0};
   // struct Pather2::point GPS_dest = {70, 10};
   // struct Pather2::point dest = getDest(GPS_src, GPS_dest);
   // std::cout << "(" << dest.x << "," << dest.y << ")" << std::endl;
   // std::cout << "First point in path: ";
   // std::cout << "(" << nextPoint.x << "," << nextPoint.y << ")";
   // std::cout << "" << std::endl;
   return 0;
}
