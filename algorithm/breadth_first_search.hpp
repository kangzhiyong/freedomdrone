#pragma once
#include <queue>
#include <string>
#include <map>
using namespace std;

enum Direction
{
    LEFT = 0,
    RIGHT,
    UP,
    DOWN
};

class BranchNode
{
protected:
    int current_node[2]{ 0, 0 };
    Direction current_direction;
};

class GirdCellCoord
{
protected:
    int x{ 0 };
    int y{ 0 };
public:
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
    
    GirdCellCoord operator+(GirdCellCoord& p)
    {
        GirdCellCoord a;
        
        return a;
    }

};

class BreadthFirstSearch
{
protected:
    GirdCellCoord start(0, 0);
    GirdCellCoord goal(4, 4);
    int grid[6][5]{ {0, 1, 0, 0, 0, 0},
                    {0, 1, 0, 0, 0, 0},
                    {0, 1, 0, 1, 0, 0},
                    {0, 0, 0, 1, 1, 0 },
                    {0, 0, 0, 1, 0, 0}};
    string sgrid[6][5] = {0};
    vector<Direction> path;
    map<Direction, GirdCellCoord> actions{
                                    {LEFT, GirdCellCoord(0, -1) } ,
                                    { RIGHT, GirdCellCoord(0, 1) } ,
                                    { UP, GirdCellCoord(-1, 0) } ,
                                    { DOWN, GirdCellCoord(1, 0) };

    string str_d(Direction d)
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
    void valid_actions(GirdCellCoord current_node, vector<Direction> valid)
    {
        // Returns a list of valid actions given a grid and current node.
        // First define a list of all possible actions
        valid.push(UP);
        valid.push(LEFT);
        valid.push(RIGHT);
        valid.push(DOWN);

        int n = 6, m = 5;
        int x = current_node.x, y = current_node.y;

        // check if the node is off the grid or it's an obstacle
        // If it is either, remove the action that takes you there
        if ((x - 1) < 0 || grid[x - 1][y] == 1)
        {
            valid.remove(UP);
        }
        if ((x + 1) > n || grid[x + 1][y] == 1)
        {
            valid.remove(DOWN);
        }
        if ((y - 1) < 0 || grid[x][y - 1] == 1)
        {
            valid.remove(LEFT);
        }
        if ((y + 1) > m || grid[x][y + 1] == 1)
        {
            valid.remove(RIGHT);
        }
    }

    // Define a function to visualize the path
    void visualize_path()
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
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 5; j++)
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

        GirdCellCoord pos(start[0], start[1]);
        // Fill in the string grid
        GirdCellCoord da;
        Direction d;
        for (int i = 0; i < path.size(); i++)
        {
            d = path[i];
            da = actions[d];
            sgrid[pos[0] + da.getX()][pos[1] + da.getY()] = str(d);
            pos += da;
        }

        sgrid[pos[0]][pos[1]] = "G";
        sgrid[start[0]][start[1]] = "S";
    }
};


// first search function
void breadth_first()
{
    // TODO : Replace the None values for
    // "queue" and "visited" with data structure objects
    // and add the start position to each
    queue<int[2]> q;
    q.push(start);
    vector<int[2]> visited;
    visited.push_back(start);

    BranchNode branch[6][5];

    bool found = false;
    int current_node0, current_node1;
    // Run loop while queue is not empty
    while (!q.empty())
    {
        // TODO : Remove the first element from the queue
        current_node = q.front();
        q.pop();

        # TODO : Check if the current
        # node corresponds to the goal state
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # TODO : Get the new nodes connected to the current node
            # Iterate through each of the new nodesand :
            # If the node has not been visited you will need to
            # 1. Mark it as visited
            # 2. Add it to the queue
            # 3. Add how you got there to the branch dictionary
            for a in valid_actions(grid, current_node) :
                # delta of performing the action
                da = a.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                if next_node not in visited :
            visited.add(next_node)
            queue.put(next_node)
            branch[next_node] = (current_node, a)

            # Now, if you found a path, retrace your steps through
            # the branch dictionary to find out how you got there!
            path = []
            if found:
            # retrace steps
            path = []
            n = goal
            while branch[n][0] != start:
            path.append(branch[n][1])
            n = branch[n][0]
            path.append(branch[n][1])

            return path[:: - 1]