//
//  breadth_first_search.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/5.
//

#include <stdio.h>
#include <iostream>
#include <set>
using namespace std;

#include "SearchAlgorithm.hpp"
#include "DroneUtils.hpp"

//int grid[ROW][COL] = {  {0, 1, 0, 0, 0, 0},
//                        {0, 0, 0, 0, 0, 0},
//                        {0, 1, 0, 0, 0, 0},
//                        {0, 0, 0, 1, 1, 0},
//                        {0, 0, 0, 1, 0, 0}};
//
//map<Direction, GirdCellCoord> actions{  {LEFT, GirdCellCoord(0, -1) } ,
//                                        { RIGHT, GirdCellCoord(0, 1) } ,
//                                        { UP, GirdCellCoord(-1, 0) } ,
//                                        { DOWN, GirdCellCoord(1, 0) } };

int g_north_size = 0;
int g_east_size = 0;
int g_alt_size = 0;

char SearchAlgorithm::str_d(Direction d)
{
    if (d == LEFT)
    {
        return '<';
    }
    else if (d == RIGHT)
    {
        return '>';
    }
    else if (d == UP)
    {
        return '^';
    }
    else if (d == DOWN)
    {
        return 'v';
    }
}

// Define a function that returns a list of valid actions
// through the grid from the current node
void SearchAlgorithm::valid_actions(GirdCellType current_node, vector<Direction> &valid)
{
    // Returns a list of valid actions given a grid and current node.
    // First define a list of all possible actions

    int n = g_east_size - 1, m = g_north_size - 1;
    int x = current_node.get(0), y = current_node.get(1);
    vector<Direction>::iterator it;
    
    // check if the node is off the grid or it's an obstacle
    // If it is either, remove the action that takes you there
    if ((x - 1) >= 0 && grid[(x - 1) * g_north_size + y] != 1)
    {
        valid.push_back(UP);
    }
    if ((x + 1) <= n && grid[(x + 1) * g_north_size + y] != 1)
    {
        valid.push_back(DOWN);
    }
    if ((y - 1) >= 0 && grid[x * g_north_size + (y - 1)] != 1)
    {
        valid.push_back(LEFT);
    }
    if ((y + 1) <= m && grid[x * g_north_size + (y + 1)] != 1)
    {
        valid.push_back(RIGHT);
    }
}

// Define a function to visualize the path
void SearchAlgorithm::visualize_path()
{
    /*
    Given a grid, pathand start position
    return visual of the path to the goal.

    'S'->start
    'G'->goal
    'O'->obstacle
    ' '->empty
    */
    // Define a grid of string characters for visualization
    char *sgrid = new char(g_east_size * g_north_size);
    for (int i = 0; i < g_east_size; i++)
    {
        for (int j = 0; j < g_north_size; j++)
        {
            if (grid[i * g_north_size + j] == 1)
            {
                sgrid[i * g_north_size + j] = 'O';
            }
            else
            {
                sgrid[i * g_north_size + j] = ' ';
            }
        }
    }

    GirdCellType pos = start;
    // Fill in the string grid
    GirdCellType da;
    Direction d;
    for (int i = path.size() - 1; i >= 0; i--)
    {
        d = path[i];
        da = actions[d];
        sgrid[int(pos.get(0) * g_north_size + pos.get(1))] = str_d(d);
        pos += da;
    }

    sgrid[int(goal.get(0) * g_north_size + goal.get(1))] = 'G';
    sgrid[int(start.get(0) * g_north_size + start.get(1))] = 'S';
    
    for (int i = 0; i < g_east_size; i++) {
        cout << "[ ";
        for (int j = 0; j < g_north_size; j++) {
            cout << " " << sgrid[i * g_north_size + j];
        }
        cout << " ]" << endl;
    }
}
    
// breadth first search function
void SearchAlgorithm::breadth_first()
{
    // TODO : Replace the None values for
    // "queue" and "visited" with data structure objects
    // and add the start position to each
    queue<GirdCellType> q;
    q.push(start);
    vector<GirdCellType> visited;
    visited.push_back(start);

    map<GirdCellType, BranchNode, MyCompare> branch;

    bool found = false;
    GirdCellType current_node;
    // Run loop while queue is not empty
    while (!q.empty())
    {
        // TODO : Remove the first element from the queue
        current_node = q.front();
        q.pop();

        // TODO : Check if the current
        // node corresponds to the goal state
        if (current_node == goal)
        {
            printf("Found a path.\r\n");
            found = true;
            break;
        }
        else
        {
            /* TODO : Get the new nodes connected to the current node
            # Iterate through each of the new nodesand :
            # If the node has not been visited you will need to
            # 1. Mark it as visited
            # 2. Add it to the queue
            # 3. Add how you got there to the branch dictionary
             */
            vector<Direction> valid;
            GirdCellType da, next_node;
            valid_actions(current_node, valid);
            for (int i = 0; i < valid.size(); i++)
            {
                // delta of performing the action
                Direction a = valid[i];
                da = actions[a];
                next_node = GirdCellType(current_node.get(0) + da.get(0), current_node.get(1) + da.get(1));
                
                if (std::find(visited.begin(), visited.end(), next_node) == visited.end())
                {
                    visited.push_back(next_node);
                    q.push(next_node);
                    branch[next_node] = BranchNode(current_node, a);
                }
            }
        }
    }
    // Now, if you found a path, retrace your steps through
    // the branch dictionary to find out how you got there!
    path.clear();
    if (found)
    {
        // retrace steps
        GirdCellType n = goal;
        while (branch[n].getCurrentNode() != start)
        {
            path.push_back(branch[n].getCurrentDirection());
            n = branch[n].getCurrentNode();
        }
        path.push_back(branch[n].getCurrentDirection());
    }
}

void SearchAlgorithm::depth_first()
{
    // TODO : Replace the None values for
    // "statck" and "visited" with data structure objects
    // and add the start position to each
    vector<GirdCellType> visited;
    map<GirdCellType, BranchNode, MyCompare> branch;
    bool found = false;
    GirdCellType current_node;    
    stack<GirdCellType> s;
    s.push(start);
 
    // Run loop while queue is not empty
    while (!s.empty())
    {
        // TODO : Remove the first element from the stack
        current_node = s.top();
        s.pop();

        // TODO : Check if the current
        // node corresponds to the goal state
        if (current_node == goal)
        {
            printf("Found a path.\r\n");
            found = true;
            break;
        }
        else
        {
            if (std::find(visited.begin(), visited.end(), current_node) == visited.end())
            {
                visited.push_back(current_node);
                vector<Direction> valid;
                GirdCellType da, next_node;
                valid_actions(current_node, valid);
                for (int i = 0; i < valid.size(); i++)
                {
                    // delta of performing the action
                    Direction a = valid[i];
                    da = actions[a];
                    next_node = GirdCellType(current_node.get(0) + da.get(0), current_node.get(1) + da.get(1));
                    if (std::find(visited.begin(), visited.end(), next_node) == visited.end())
                    {
                        s.push(next_node);
                        branch[next_node] = BranchNode(current_node, a);
                    }
                }
            }
        }
    }
    // Now, if you found a path, retrace your steps through
    // the branch dictionary to find out how you got there!
    path.clear();
    if (found)
    {
        // retrace steps
        GirdCellType n = goal;
        while (branch[n].getCurrentNode() != start)
        {
            path.push_back(branch[n].getCurrentDirection());
            n = branch[n].getCurrentNode();
        }
        path.push_back(branch[n].getCurrentDirection());
    }
}

float heuristic(GirdCellType p, GirdCellType g)
{
    return (p - g).getCost();
}

int heuristic_func(GirdCellType p, GirdCellType g)
{
    return abs(p.get(0) - g.get(0)) + abs(p.get(1) - g.get(1));
}
void SearchAlgorithm::a_star()
{
    int path_cost, current_cost = 0;
    set<GirdCellType, MyCompare> visited;
    map<GirdCellType, BranchNode, MyCompare> branch;
    bool found = false;
    GirdCellType current_node;   
    priority_queue<GirdCellType> q;
    start.queue_cost = 0;
    q.push(start);
    visited.insert(start);

    while (!q.empty())
    {
        current_node = q.top();
        q.pop();
        if (current_node == start)
        {
            current_cost = 0;
        }
        else
        {
            current_cost = branch[current_node].getCost();
        }
        if (current_node == goal)
        {
            cout << "Found a path" << endl;
            found = true;
            break;
        }
        else
        {
            int branch_cost, queue_cost = 0;
            vector<Direction> valid;
            GirdCellType da, next_node;
            valid_actions(current_node, valid);
            for (int i = 0; i < valid.size(); i++)
            {
                // delta of performing the action
                Direction a = valid[i];
                da = actions[a];
                next_node = GirdCellType(current_node.get(0) + da.get(0), current_node.get(1) + da.get(1));
                branch_cost = current_cost + da.getCost();
                next_node.queue_cost = branch_cost + heuristic_func(next_node, goal);
                if ( visited.find(next_node) == visited.end())
                {
                    visited.insert(next_node);
                    q.push(next_node);
                    branch[next_node] = BranchNode( branch_cost, current_node, a);
                }
            }
        }
    }
    path.clear();
    if (found)
    {
        // retrace steps
        GirdCellType n = goal;
        path_cost = branch[n].getCost();
        path_points.push_back(goal.m_point);
        while (branch[n].getCurrentNode() != start)
        {
            path_points.push_back(branch[n].getCurrentNode().m_point);
            path.push_back(branch[n].getCurrentDirection());
            n = branch[n].getCurrentNode();
        }
        path_points.push_back(branch[n].getCurrentNode().m_point);
        path.push_back(branch[n].getCurrentDirection());
    }
}

void SearchAlgorithm::a_start_graph(FreeGraph<float, 3> g)
{
    if (start.m_point == goal.m_point)
    {
        return;
    }
    int path_cost, current_cost = 0;
    set<GirdCellType, MyCompare> visited;
    map<GirdCellType, BranchNode, MyCompare> branch;
    bool found = false;
    GirdCellType current_node;
    priority_queue<GirdCellType> q;
    start.queue_cost = 0;
    q.push(start);
    visited.insert(start);

    while (!q.empty())
    {
        current_node = q.top();
        q.pop();
        if (current_node == start)
        {
            current_cost = 0;
        }
        else
        {
            current_cost = branch[current_node].getCost();
        }
        if (current_node == goal)
        {
            cout << "Found a path" << endl;
            found = true;
            break;
        }
        else
        {
            int new_cost, cost = 0;
            FreeGraph<float, 3>::EdgesType edges = g.edges(current_node.m_point);
            FreeGraph<float, 3>::EdgesType::iterator eit = edges.begin();
            GirdCellType next_node;
            while (eit != edges.end())
            {
                next_node = GirdCellType(eit->first);
                cost = atoi(eit->second["weight"].c_str());
                new_cost = current_cost + cost + heuristic(next_node, goal);
                next_node.queue_cost = new_cost;
                if (visited.find(next_node) == visited.end())
                {
                    visited.insert(next_node);
                    q.push(next_node);
                    branch[next_node] = BranchNode(new_cost, current_node);
                }
                eit++;
            }
        }
    }
    path_points.clear();
    if (found)
    {
        // retrace steps
        GirdCellType n = goal;
        path_cost = branch[n].getCost();
        path_points.push_back(goal.m_point);
        map<GirdCellType, BranchNode, MyCompare>::iterator it = branch.find(n);
        while ( it != branch.end() && branch[n].getCurrentNode() != start)
        {
            path_points.push_back(branch[n].getCurrentNode().m_point);
            n = branch[n].getCurrentNode();
        }
        path_points.push_back(branch[n].getCurrentNode().m_point);
        reverse(path_points.begin(), path_points.end());
    }
}

//Implementation of Bresenham's line drawing algorithm
//See en.wikipedia.org/wiki/Bresenham's_line_algorithm
bool SearchAlgorithm::bresenham(V3F p0, V3F p1)
{
    /*
    Yield integer coordinates on the line from (x0, y0) to (x1, y1).
    Input coordinates should be integers.
    The result will contain both the start and the end point.
     */
    int dx = p1[0] - p0[0];
    int dy = p1[1] - p0[1];
    
    int xsign = dx > 0 ? 1 : -1;
    int ysign = dy > 0 ? 1 : -1;
    
    dx = abs(dx);
    dy = abs(dy);
    
    int xx, xy, yx, yy;
    if (dx > dy) {
        xx = xsign;
        xy = 0;
        yx = 0;
        yy = ysign;
    }
    else
    {
        int tmp = dx;
        dx = dy;
        dy = tmp;
        xx = 0;
        xy = ysign;
        yx = xsign;
        yy = 0;
    }
    
    int D = 2 * dy - dx;
    int y = 0;
    for (int x = 0; x < (dx + 1); x++) {
        if (grid[(p0[0] + x*xx + y*yx) * g_north_size + p0[1] + x*xy + y*yy] == 1) {
            return false;
        }
        if (D >= 0) {
            y += 1;
            D -= 2 * dx;
        }
        D += 2 * dy;
    }
    return true;
}

bool SearchAlgorithm::collinearity(V3F p1, V3F p2, V3F p3)
{
    // TODO: Calculate the determinant of the matrix using integer arithmetic
    // TODO: Set collinear to True if the determinant is equal to zero
    return (p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])) == 0;
}

void SearchAlgorithm::prune_path_by_collinearity(vector<V3F> path, vector<V3F> &pruned_path)
{
    pruned_path = path;
    int i = 2;
    while (i < pruned_path.size())
    {
        V3F p1 = pruned_path[i - 2];
        V3F p2 = pruned_path[i - 1];
        V3F p3 = pruned_path[i];
        
        /*
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        */
        if (collinearity(p1, p2, p3))
        {
            /*
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
             */
            pruned_path.erase(std::find(pruned_path.begin(), pruned_path.end(), p2));
        }
        else
        {
            i += 1;
        }
    }
}

void SearchAlgorithm::prune_path_by_bresenham(vector<V3F> path, vector<V3F> &pruned_path)
{
    pruned_path = path;
    int i = 1;
    while (i < pruned_path.size())
    {
        V3F p1 = pruned_path[i - 1];
        V3F p2 = pruned_path[i];
        
        /*
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        */
        if (bresenham(p1, p2))
        {
            /*
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
             */
            pruned_path.erase(std::find(pruned_path.begin(), pruned_path.end(), p2));
        }
        else
        {
            i += 1;
        }
    }
    pruned_path.push_back({start.get(0), start.get(1)});
}
