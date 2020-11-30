#pragma once
#include <queue>
#include <string>
#include <map>
#include <vector>
#include <stack>
using namespace std;

#include "Point.hpp"
#include "FreeGraph.hpp"

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
    typedef Point<coordinate_type, dimensions> PointType;
    Point<coordinate_type, dimensions> m_point;
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
    GirdCellCoord(float _x, float _y)
    {
        m_point = PointType({ _x, _y, 0 });
    }

    coordinate_type get(size_t index)
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
        return GirdCellCoord(m_point + p.m_point);
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

typedef GirdCellCoord<float, 3> GirdCellType;

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
    vector<V3F> path_points;
    vector<float> grid;
    map<Direction, GirdCellType> actions;
public:
    SearchAlgorithm(){}
    SearchAlgorithm(GirdCellType s, GirdCellType g, vector<float> _grid)
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
    void a_start_graph(FreeGraph<float, 3> g);
    vector< V3F > get_path_points()
    {
        return path_points;
    }
    bool bresenham(V3F p0, V3F p1);
    bool collinearity(V3F p1, V3F p2, V3F p3);
    void prune_path_by_collinearity(vector<V3F> path, vector<V3F> &pruned_path);
    void prune_path_by_bresenham(vector<V3F> path, vector<V3F> &pruned_path);
};

extern int g_north_size;
extern int g_east_size;
extern int g_alt_size;

struct MyCompare {  //Function Object
    bool operator()(const GirdCellType& p1, const GirdCellType& p2) const {
        return (p1.m_point.get(0) * g_north_size + p1.m_point.get(1)) < (p2.m_point.get(0) * g_north_size + p2.m_point.get(1));
    }
};
