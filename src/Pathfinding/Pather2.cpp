//#include "ObstacleMap.h"
#include "Pather2.h"
#include <vector>
#include <queue>
#include <iostream>

//matrix size
#define ROW 5
#define COL 5


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

Pather2::point getPath(bool map[][5], Pather2::point dest){

    Pather2::point src = {2,2};

    //check if src and dest are represented with 1 not 0
    if ((map[src.x][src.y]) || (map[dest.x][dest.y])){
        struct Pather2::point error = {-1, -1};
        //todo: change destination if destination is blocked
        return error;
    }

    //create 2D boolean array and use memset to set all values as false
    //bool visited[21][21];
    //memset(visited, false, sizeof visited); 

    // for (int i = 0; i < ROW; i++) {
    //     for (int j = 0; j < COL; j++) {
    //         if (map[i][j]) {
    //             visited[i][j] = true;
    //         }
    //     }
    // } 

    //set source as visited, create node with dist 0 from src and push to queue of nodes to be checked
    //visited[src.x][src.y] = true;
    map[src.x][src.y] = true;
    std::queue<Pather2::queueNode> q;
    std::queue<Pather2::point> newQueue;
    newQueue.push(src);
    Pather2::queueNode s = {src, 0, newQueue};
    q.push(s);

    // q is the queue of points that are waiting to be checked
    // curr.path is the path leading up to that point

    while (!q.empty()){
        //get front node in nodes to be checked and make it current
        Pather2::queueNode curr = q.front();
        Pather2::point pt = curr.pt;

        //check if current node is destination node
        if ((pt.x == dest.x) && (pt.y == dest.y)){
            //return first node after source in path to destination
            std::queue<Pather2::point> temp = curr.path;
            temp.pop();
            std::cout << "Final path: ";
            print_point(temp);
            return temp.front(); 
        }

        // pop current node off nodes to be checked
        q.pop();
 
        // iterate through adj nodes
        for (int i = 0; i < 8; i++){
            int row = pt.x + rowNum[i];
            int col = pt.y + rowCol[i];
            

            // check if adj node is not out of bounds and is not an obstacle
            if (isValid(row, col) && !map[row][col]){
                // set adj node as visited and push it to nodes to be checked
                struct Pather2::point test = {row, col};
                std::queue<Pather2::point> addAdjPointToPath = curr.path;
                addAdjPointToPath.push(test);
                Pather2::queueNode adjNode = {{row, col}, curr.dist + 1, addAdjPointToPath};
                map[adjNode.pt.x][adjNode.pt.y] = true;
                q.push(adjNode);
            }
        }

    }
    return src;
}

// Pather2::Pather2(ObstacleMap obstacle_map)
// {
//     map = obstacle_map;   
// }

int main() {
    bool map[ROW][COL] = {{true, false, true, false, true},
                         {true, true, true, true, false},
                         {true, false, false, false, false},
                         {true, false, false, true, true},
                         {false, false, false, false, false}};
    struct Pather2::point destination = {4, 4};
    struct Pather2::point nextPoint = getPath(map, destination);
    std::cout << "First point in path: ";
    std::cout << "(" << nextPoint.x << "," << nextPoint.y << ")";
    std::cout << "" << std::endl;
    return 0;
}