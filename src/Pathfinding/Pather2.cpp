#include "Map.h"
#include "Pather.h"
#include <vector>
#include <queue>

//matrix size
#define ROW 5
#define COL 5

//check if node is valid
bool isValid(int x, int y) {
    return (x >= 0) && (x < ROW) && (y >= 0) && (y < COL);
}

int rowNum[] = {-1,0,0,1};
int rowCol[] = {0,-1,-1,0};

Pather::point BFS(int mat[ROW][COl], Pather::point src, Pather::point dest){

    //check if src and dest are represented with 1 not 0
    if (!mat[src.x][src.y] || !mat[dest.x][dest.y]){
        return -1;
    }

    //create 2D boolean array and use memset to set all values as false
    bool visited[ROW][COL];
    memset(visited, false, sizeof visited); 

    //set source as visited, create node with dist 0 from src and push to queue of nodes to be checked
    visited[src.x][src.y] = true;
    std::queue<Pather::queueNode> q;
    Pather::queueNode s = {src, 0, {0,0}};
    q.push(s);

    while (!q.empty()){
        
        //get front node in nodes to be checked and make it current
        Pather::queueNode curr = q.front();
        Pather::point pt = curr.pt;

        //check if current node is destination node
        if (pt.x == dest.x) && (pt.y == dest.y){
            //return first node after source in path to destination
            std::queue<Pather::point> temp = curr.path;
            temp.pop();
            return temp.pop();
        }

        //pop current node off nodes to be checked
        q.pop();

        //iterate through adj nodes
        for (int i = 0, i < 4, i++){
            int row = pt.x + rowNum[i];
            int col = pt.y + rowCol[i];
            curr.path.push({curr.pt.x, curr.pt.y});
            Pather::queueNode adjNode = {{row, col}, curr.dist + 1, curr.path};

            //check if adj node is not out of bounds and is not an obstacle
            if (isValid(adjNode.pt.x) && isValid(adjNode.pt.y) && !visited[adjNode.pt.x][adjNode.pt.y]){
                //set adj node as visited and push it to nodes to be checked
                visited[adjNode.pt.x][adjNode.pt.y] = true;
                q.push(adjNode);
            }
        }

    }
    return src;
}