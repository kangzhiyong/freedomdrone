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

#include "search_algorithm.hpp"
#include "free_utils.hpp"

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
void SearchAlgorithm::valid_actions(GirdCellCoord current_node, vector<Direction> &valid)
{
    // Returns a list of valid actions given a grid and current node.
    // First define a list of all possible actions

    int n = g_east_size - 1, m = g_north_size - 1;
    int x = current_node.getX(), y = current_node.getY();
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

    GirdCellCoord pos = start;
    // Fill in the string grid
    GirdCellCoord da;
    Direction d;
    for (int i = path.size() - 1; i >= 0; i--)
    {
        d = path[i];
        da = actions[d];
        sgrid[pos.getX() * g_north_size + pos.getY()] = str_d(d);
        pos += da;
    }

    sgrid[goal.getX() * g_north_size + goal.getY()] = 'G';
    sgrid[start.getX() * g_north_size + start.getY()] = 'S';
    
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
    queue<GirdCellCoord> q;
    q.push(start);
    vector<GirdCellCoord> visited;
    visited.push_back(start);

    map<GirdCellCoord, BranchNode, MyCompare> branch;

    bool found = false;
    GirdCellCoord current_node;
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
            GirdCellCoord da, next_node;
            valid_actions(current_node, valid);
            for (int i = 0; i < valid.size(); i++)
            {
                // delta of performing the action
                Direction a = valid[i];
                da = actions[a];
                next_node = GirdCellCoord(current_node.getX() + da.getX(), current_node.getY() + da.getY());
                
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
        GirdCellCoord n = goal;
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
    vector<GirdCellCoord> visited;
    map<GirdCellCoord, BranchNode, MyCompare> branch;
    bool found = false;
    GirdCellCoord current_node;    
    stack<GirdCellCoord> s;
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
                GirdCellCoord da, next_node;
                valid_actions(current_node, valid);
                for (int i = 0; i < valid.size(); i++)
                {
                    // delta of performing the action
                    Direction a = valid[i];
                    da = actions[a];
                    next_node = GirdCellCoord(current_node.getX() + da.getX(), current_node.getY() + da.getY());
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
        GirdCellCoord n = goal;
        while (branch[n].getCurrentNode() != start)
        {
            path.push_back(branch[n].getCurrentDirection());
            n = branch[n].getCurrentNode();
        }
        path.push_back(branch[n].getCurrentDirection());
    }
}

float heuristic(GirdCellCoord p, GirdCellCoord g)
{
    return (p - g).getCost();
}

int heuristic_func(GirdCellCoord p, GirdCellCoord g)
{
    return abs(p.getX() - g.getX()) + abs(p.getY() - g.getY());
}
void SearchAlgorithm::a_star()
{
    int path_cost, current_cost = 0;
    set<GirdCellCoord, MyCompare> visited;
    map<GirdCellCoord, BranchNode, MyCompare> branch;
    bool found = false;
    GirdCellCoord current_node;   
    priority_queue<GirdCellCoord> q;
    q.push((0, start));
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
            GirdCellCoord da, next_node;
            valid_actions(current_node, valid);
            for (int i = 0; i < valid.size(); i++)
            {
                // delta of performing the action
                Direction a = valid[i];
                da = actions[a];
                next_node = GirdCellCoord(current_node.getX() + da.getX(), current_node.getY() + da.getY());
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
        GirdCellCoord n = goal;
        path_cost = branch[n].getCost();
        path_points.push_back({goal.getX(), goal.getY()});
        while (branch[n].getCurrentNode() != start)
        {
            path_points.push_back({branch[n].getCurrentNode().getX(), branch[n].getCurrentNode().getY()});
            path.push_back(branch[n].getCurrentDirection());
            n = branch[n].getCurrentNode();
        }
        path_points.push_back({branch[n].getCurrentNode().getX(), branch[n].getCurrentNode().getY()});
        path.push_back(branch[n].getCurrentDirection());
    }
}

//Implementation of Bresenham's line drawing algorithm
//See en.wikipedia.org/wiki/Bresenham's_line_algorithm
bool SearchAlgorithm::bresenham(point<int, 2> p0, point<int, 2> p1)
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

bool SearchAlgorithm::collinearity(point<int, 2> p1, point<int, 2> p2, point<int, 2> p3)
{
    // TODO: Calculate the determinant of the matrix using integer arithmetic
    // TODO: Set collinear to True if the determinant is equal to zero
    return (p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])) == 0;
}

void SearchAlgorithm::prune_path_by_collinearity(vector<point<int, 2>> path, vector<point<int, 2>> &pruned_path)
{
    pruned_path = path;
    int i = 2;
    while (i < pruned_path.size())
    {
        point<int, 2> p1 = pruned_path[i - 2];
        point<int, 2> p2 = pruned_path[i - 1];
        point<int, 2> p3 = pruned_path[i];
        
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

void SearchAlgorithm::prune_path_by_bresenham(vector<point<int, 2>> path, vector<point<int, 2>> &pruned_path)
{
    pruned_path = path;
    int i = 1;
    while (i < pruned_path.size())
    {
        point<int, 2> p1 = pruned_path[i - 1];
        point<int, 2> p2 = pruned_path[i];
        
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
    pruned_path.push_back({start.getX(), start.getY()});
}
