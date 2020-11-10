//
//  breadth_first_search.cpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/5.
//

#include <stdio.h>
#include <iostream>
using namespace std;

#include "search_algorithm.hpp"

int grid[ROW][COL] = {  {0, 1, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 0, 1, 1, 0},
                        {0, 0, 0, 1, 0, 0}};

map<Direction, GirdCellCoord> actions{  {LEFT, GirdCellCoord(0, -1) } ,
                                        { RIGHT, GirdCellCoord(0, 1) } ,
                                        { UP, GirdCellCoord(-1, 0) } ,
                                        { DOWN, GirdCellCoord(1, 0) } };

string SearchAlgorithm::str_d(Direction d)
{
    if (d == LEFT)
    {
        return "<";
    }
    else if (d == RIGHT)
    {
        return ">";
    }
    else if (d == UP)
    {
        return "^";
    }
    else if (d == DOWN)
    {
        return "v";
    }
}

// Define a function that returns a list of valid actions
// through the grid from the current node
void SearchAlgorithm::valid_actions(GirdCellCoord current_node, vector<Direction> &valid)
{
    // Returns a list of valid actions given a grid and current node.
    // First define a list of all possible actions

    int n = ROW - 1, m = COL - 1;
    int x = current_node.getX(), y = current_node.getY();
    vector<Direction>::iterator it;
    valid.push_back(UP);
    valid.push_back(DOWN);
    valid.push_back(LEFT);
    valid.push_back(RIGHT);
    
    // check if the node is off the grid or it's an obstacle
    // If it is either, remove the action that takes you there
    if ((x - 1) < 0 || grid[x - 1][y] == 1)
    {
        valid.erase(find(valid.begin(), valid.end(), UP));
    }
    if ((x + 1) > n && grid[x + 1][y] == 1)
    {
        valid.erase(find(valid.begin(), valid.end(), DOWN));
    }
    if ((y - 1) < 0 && grid[x][y - 1] == 1)
    {
        valid.erase(find(valid.begin(), valid.end(), LEFT));
    }
    if ((y + 1) > m && grid[x][y + 1] == 1)
    {
        valid.erase(find(valid.begin(), valid.end(), RIGHT));
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
    string sgrid[ROW][COL];
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            if (grid[i][j] == 1)
            {
                sgrid[i][j] = "O";
            }
            else
            {
                sgrid[i][j] = " ";
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
        sgrid[pos.getX()][pos.getY()] = str_d(d);
        pos += da;
    }

    sgrid[goal.getX()][goal.getY()] = "G";
    sgrid[start.getX()][start.getY()] = "S";
    
    for (int i = 0; i < ROW; i++) {
        cout << "[ ";
        for (int j = 0; j < COL; j++) {
            cout << " " << sgrid[i][j];
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

void SearchAlgorithm::a_star()
{
    float path_cost, current_cost = 0;
    vector<GirdCellCoord> visited;
    map<GirdCellCoord, BranchNode, MyCompare> branch;
    bool found = false;
    GirdCellCoord current_node;   
    priority_queue<GirdCellCoord> q;
    q.push((0, start));
    visited.push_back(start);
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
            float branch_cost, queue_cost = 0;
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
                next_node.queue_cost = branch_cost + heuristic(next_node, goal);

                if (std::find(visited.begin(), visited.end(), next_node) == visited.end())
                {
                    visited.push_back(next_node);
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
