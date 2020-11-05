#pragma once
#include <queue>
#include <string>
#include <map>
#include <vector>
using namespace std;

#define ROW 5
#define COL 6

enum Direction
{
    LEFT = 0,
    RIGHT,
    UP,
    DOWN
};

class GirdCellCoord
{
public:
    int x{ 0 };
    int y{ 0 };

    GirdCellCoord(){}
    GirdCellCoord(int _x, int _y)
    {
        x = _x;
        y = _y;
    }
    int getX()
    {
        return x;
    }
    int getY()
    {
        return y;
    }
    
    void setX(int _x)
    {
        x = _x;
    }
    
    void setY(int _y)
    {
        y = _y;
    }
    
    GirdCellCoord operator+(GirdCellCoord& p)
    {
        GirdCellCoord a(0, 0);
        a.setX(getX() + p.getX());
        a.setY(getY() + p.getY());
        return a;
    }

    void operator+=(GirdCellCoord& p)
    {
        x += p.getX();
        y += p.getY();
    }
    
    bool operator ==(const GirdCellCoord& p)
    {
        return x == p.x && y == p.y;
    }
    
    bool operator !=(const GirdCellCoord& p)
    {
        return x != p.x || y != p.y;
    }
};

struct MyCompare{  //Function Object
    bool operator()(const GirdCellCoord &p1, const GirdCellCoord &p2) const{
        return (p1.x * COL + p1.y) < (p2.x * COL + p2.y);
    }
};

class BranchNode
{
public:
    BranchNode(){}
    BranchNode(GirdCellCoord c, Direction d)
    {
        current_node = c;
        current_direction = d;
    }
    GirdCellCoord getCurrentNode()
    {
        return current_node;
    }
    Direction getCurrentDirection()
    {
        return current_direction;
    }
protected:
    GirdCellCoord current_node;
    Direction current_direction;
};

class BreadthFirstSearch
{
protected:
    vector<Direction> path;
public:
    BreadthFirstSearch(){}
    string str_d(Direction d);

    // Define a function that returns a list of valid actions
    // through the grid from the current node
    void valid_actions(GirdCellCoord current_node, vector<Direction> &valid);

    // Define a function to visualize the path
    void visualize_path();
        
    // breadth first search function
    void breadth_first();
};
