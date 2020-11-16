#pragma once
#include <queue>
#include <string>
#include <map>
#include <vector>
#include <stack>
using namespace std;

#include "free_point.hpp"
#include "free_graph.hpp"

//#define ROW 5
//#define COL 6

enum Direction
{
    LEFT = 0,
    RIGHT,
    UP,
    DOWN
};

template<typename coordinate_type, size_t dimensions>
class GirdCellCoord
{
public:
    typedef point<coordinate_type, dimensions> PointType;
    point<coordinate_type, dimensions> m_point;
    int queue_cost{ 0 };//起点到该点的cost+该点到终点的预估最小值(c=h+g)

    GirdCellCoord(){}
    GirdCellCoord(int _x, int _y, int _z, int cost)
    {
        m_point = PointType({_x, _y, _z});
        queue_cost = cost;
    }
    GirdCellCoord(PointType p)
    {
        m_point = p;
    }
    GirdCellCoord(PointType p, int cost)
    {
        m_point = p;
        queue_cost = cost;
    }
    GirdCellCoord(int _x, int _y)
    {
        m_point = PointType({ _x, _y, 0 });
    }

    int get(size_t index)
    {
        return m_point.get(index);
    }
    
    float getCost()
    {
        float dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            float d = get(i);
            dist += d * d;
        }
        return sqrt(dist);
    }

    GirdCellCoord operator+(GirdCellCoord& p)
    {
        return GirdCellCoord(a.m_point + p.m_point);
    }

    GirdCellCoord operator-(GirdCellCoord& p)
    {
        return GirdCellCoord(m_point - p.m_point);
    }

    void operator+=(GirdCellCoord& p)
    {
        m_point = m_point + p.m_point;
    }
    
    bool operator ==(const GirdCellCoord& p)
    {
        return m_point == p.m_point;
    }
    
    bool operator !=(const GirdCellCoord& p)
    {
        return !(m_point == p.m_point);
    }

    bool operator<(const GirdCellCoord& a) const
    {
        return queue_cost > a.queue_cost;
    }
};

typedef GirdCellCoord<int, 3> GirdCellType;

class BranchNode
{
public:
    BranchNode(){}
    BranchNode(GirdCellType c, Direction d)
    {
        current_node = c;
        current_direction = d;
    }
    BranchNode(int _cost, GirdCellType c, Direction d)
    {
        cost = _cost;
        current_node = c;
        current_direction = d;
    }

    BranchNode(int _cost, GirdCellType c)
    {
        cost = _cost;
        current_node = c;
    }

    GirdCellType getCurrentNode()
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
    GirdCellType current_node;
    Direction current_direction;
};

class SearchAlgorithm
{
protected:
    GirdCellType start;
    GirdCellType goal;
    vector<Direction> path;
    vector<point<int, 3> > path_points;
    vector<int> grid;
    map<Direction, GirdCellType> actions;
public:
    SearchAlgorithm(){}
    SearchAlgorithm(GirdCellType s, GirdCellType g, vector<int> _grid)
    {
        start = s;
        goal = g;
        grid = _grid;
        actions.insert({ LEFT, GirdCellType(0, -1) });
        actions.insert({ RIGHT, GirdCellType(0, 1) });
        actions.insert({ UP, GirdCellType(-1, 0) });
        actions.insert({ DOWN, GirdCellType(1, 0) });
    }

    SearchAlgorithm(GirdCellType s, GirdCellType g)
    {
        start = s;
        goal = g;
    }

    char str_d(Direction d);

    // Define a function that returns a list of valid actions
    // through the grid from the current node
    void valid_actions(GirdCellType current_node, vector<Direction> &valid);

    // Define a function to visualize the path
    void visualize_path();
        
    // breadth first search function
    void breadth_first();

    // Depth-first search (DFS)
    void depth_first();

    // A-Star search
    void a_star();
    void a_start_graph(FreeGraph<int, 3> g);
    vector< point<int, 3> > get_path_points()
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
    bool operator()(const GirdCellType& p1, const GirdCellType& p2) const {
        return (p1.m_point.get(0) * g_north_size + p1.m_point.get(1)) < (p2.m_point.get(0) * g_north_size + p2.m_point.get(1));
    }
};
