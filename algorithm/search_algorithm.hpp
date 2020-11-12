#pragma once
#include <queue>
#include <string>
#include <map>
#include <vector>
#include <stack>
using namespace std;

#include "free_point.hpp"

//#define ROW 5
//#define COL 6

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
    int queue_cost{ 0 };//起点到该点的cost+该点到终点的预估最小值(c=h+g)

    GirdCellCoord(){}
    GirdCellCoord(int _x, int _y, int cost)
    {
        x = _x;
        y = _y;
        queue_cost = cost;
    }
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
    
    float getCost()
    {
        return sqrt(pow(x, 2) + pow(y, 2));
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

    GirdCellCoord operator-(GirdCellCoord& p)
    {
        GirdCellCoord a(0, 0);
        a.setX(getX() - p.getX());
        a.setY(getY() - p.getY());
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

    bool operator<(const GirdCellCoord& a) const
    {
        return queue_cost > a.queue_cost;
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
    BranchNode(int _cost, GirdCellCoord c, Direction d)
    {
        cost = _cost;
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
    float getCost()
    {
        return cost;
    }
protected:
    int cost{ 0 };
    GirdCellCoord current_node;
    Direction current_direction;
};

class SearchAlgorithm
{
protected:
    GirdCellCoord start;
    GirdCellCoord goal;
    vector<Direction> path;
    vector<point<int, 2> > path_points;
    vector<int> grid;
    map<Direction, GirdCellCoord> actions;
public:
    SearchAlgorithm(){}
    SearchAlgorithm(GirdCellCoord s, GirdCellCoord g, vector<int> _grid)
    {
        start = s;
        goal = g;
        grid = _grid;
        actions.insert({ LEFT, GirdCellCoord(0, -1) });
        actions.insert({ RIGHT, GirdCellCoord(0, 1) });
        actions.insert({ UP, GirdCellCoord(-1, 0) });
        actions.insert({ DOWN, GirdCellCoord(1, 0) });
    }
    char str_d(Direction d);

    // Define a function that returns a list of valid actions
    // through the grid from the current node
    void valid_actions(GirdCellCoord current_node, vector<Direction> &valid);

    // Define a function to visualize the path
    void visualize_path();
        
    // breadth first search function
    void breadth_first();

    // Depth-first search (DFS)
    void depth_first();

    // A-Star search
    void a_star();
    vector< point<int, 2> > get_path_points()
    {
        return path_points;
    }
    bool bresenham(point<int, 2> p0, point<int, 2> p1);
    bool collinearity(point<int, 2> p1, point<int, 2> p2, point<int, 2> p3);
    void prune_path_by_collinearity(vector<point<int, 2>> path, vector<point<int, 2>> &pruned_path);
    void prune_path_by_bresenham(vector<point<int, 2>> path, vector<point<int, 2>> &pruned_path);
};

extern int g_north_size;
extern int g_east_size;
extern int g_alt_size;

struct MyCompare {  //Function Object
    bool operator()(const GirdCellCoord& p1, const GirdCellCoord& p2) const {
        return (p1.x * g_north_size + p1.y) < (p2.x * g_north_size + p2.y);
    }
};
