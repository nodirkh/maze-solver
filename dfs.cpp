/* *
 * Implementation:
 *
 * Here, the original DFS algorithm is used to find the exit coordinates.
 * Once the exit coordinates are found, the agent restarts the exploration phase and
 * starts seeking the shortest path. To find the exit node coordinates, DFS uses a stack data structure.
 * Each time, a neighboring non-wall cell is pushed to the stack top. Then the algorithm proceeds to go
 * in one direction, until it reaches "the deepest point". Then, it selects the other direction to follow
 * and goes by it.
 *
 * As a shortest path seeker, the cost function is used.
 * Knowing the start node coordinates and the end node coordinates, agent finds the Manhattan distance
 * between the nodes and selects the smallest values to construct the shortest path.
 *
 * To distinguish between the exploration and path seeking phases, the bool flag is created.
 *
 * */

#include <iostream>
#include <list>
#include <string>
#include <cstdlib>
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <map>
#if __has_include(<optional>)
#include <optional>
namespace stdx
{
    using namespace ::std;
}
#elif __has_include(<experimental/optional>)
#include <experimental/optional>
namespace stdx
{
    using namespace ::std;
    using namespace ::std::experimental;
}
#else
#error <experimental/optional> and <optional> not found
#endif

using namespace stdx;
// ---------------------------------------------------------------------

#define MAX_SIZE 300

class Coordinate
{

    int x, y;

public:
    Coordinate(int x, int y) : x(x), y(y) {}

    int getX() const
    {
        return x;
    }

    int getY() const
    {
        return y;
    }

    bool operator==(const Coordinate &rhs) const
    {
        return x == rhs.x && y == rhs.y;
    }

    bool operator!=(const Coordinate &rhs) const
    {
        return !(rhs == *this);
    }
};

class DepthFirstSearch
{
private:
    stack<Coordinate> dfs;                                                                                                                // a stack to implement the dfs algorithm
    set<pair<int, int>> visited;                                                                                                          // check if the node is visited to avoid loops
    map<vector<int>, vector<int>> path;                                                                                                   // trace the path in a child:parent manner
    priority_queue<pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<pair<double, pair<int, int>>>> pathfinder; // shortest path calculator
    map<vector<int>, double> cost_map;                                                                                                    // cost from node to end
    map<vector<int>, int> distance_from_start;                                                                                            // use cost-function
    int x;                                                                                                                                // track current x
    int y;                                                                                                                                // track current y
    bool flag;                                                                                                                            // flag to check if the exit is found by dfs
    vector<int> actual_exit;                                                                                                              // save the exit node's coordinates

public:
    DepthFirstSearch(int size_x, int size_y)
    {

        // initialize start values
        x = 0;
        y = 0;
        flag = false;
        actual_exit = {0, 0};
    }

    double calculateDistance(Coordinate enter, Coordinate exit)
    {
        return abs(enter.getX() - exit.getX()) + abs(enter.getY() - exit.getY()); // calculate the distance from the node to exit
    }

    optional<Coordinate> move(bool isExit, bool hasWallSouth, bool hasWallNorth, bool hasWallEast, bool hasWallWest)
    {

        if (!flag) // check if the exit is found
        {
            if (isExit) // if the node exit is found, proceed to shortest-path construction
            {
                actual_exit[0] = x; // save the end X
                actual_exit[1] = y; // save the end Y
                x = 0;              // reset the values of X and Y
                y = 0;
                flag = true; // mark the exit as found
                distance_from_start[{x, y}] = 0;
                return make_optional<Coordinate>(x, y); // restart exploration
            }

            vector<int> parent = {x, y}; // current/parent node

            if (!hasWallSouth)
            {
                // check the wall below

                auto child = Coordinate(x, y + 1);                                                  // create child (neighbor) node
                if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been visited
                {
                    dfs.push(child);                                                 // push the child node
                    visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark as seen
                }
            }
            if (!hasWallNorth)
            {
                // check the wall above

                auto child = Coordinate(x, y - 1);
                if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been visited
                {
                    dfs.push(child);                                                 // push the child node
                    visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark as seen
                }
            }
            if (!hasWallEast)
            {
                // check the wall to the right

                auto child = Coordinate(x + 1, y);
                if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been visited
                {
                    dfs.push(child);                                                 // push the child node
                    visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark as seen
                }
            }
            if (!hasWallWest)
            {
                // check the wall to the left

                auto child = Coordinate(x - 1, y);
                if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been visited
                {
                    dfs.push(child);                                                 // push the child node
                    visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark as seen
                }
            }

            auto current_coordinate = dfs.top(); // get the next node
            x = current_coordinate.getX();       // set the X and Y values to continue iteration
            y = current_coordinate.getY();
            dfs.pop();
        }
        else
        {
            if (isExit) // if the exit is found in shortest path, quit searching
                return nullopt;
            vector<int> parent = {x, y};

            if (!hasWallSouth)
            {
                auto child = Coordinate(x, y + 1);
                int tentative = distance_from_start[parent] + 1;                                 // check the cost it takes to move to the neighboring node
                vector<int> query = {child.getX(), child.getY()};                                // create a searching query
                if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
                {
                    path[query] = parent;                                                                               // if the new cost to move is less than before, change the parent
                    distance_from_start[query] = tentative;                                                             // change the cost
                    cost_map[query] = tentative + calculateDistance(child, Coordinate(actual_exit[0], actual_exit[1])); // update the cost to end node
                    pathfinder.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY())));                 // update the priority queue
                }
            }
            if (!hasWallNorth)
            {
                auto child = Coordinate(x, y - 1);
                int tentative = distance_from_start[parent] + 1;                                 // check the cost it takes to move to the neighboring node
                vector<int> query = {child.getX(), child.getY()};                                // create a searching query
                if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
                {
                    path[query] = parent;                                                                               // if the new cost to move is less than before, change the parent
                    distance_from_start[query] = tentative;                                                             // change the cost
                    cost_map[query] = tentative + calculateDistance(child, Coordinate(actual_exit[0], actual_exit[1])); // update the cost to end node
                    pathfinder.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY())));                 // update the priority queue
                }
            }
            if (!hasWallEast)
            {
                auto child = Coordinate(x + 1, y);
                int tentative = distance_from_start[parent] + 1;                                 // check the cost it takes to move to the neighboring node
                vector<int> query = {child.getX(), child.getY()};                                // create a searching query
                if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
                {
                    path[query] = parent;                                                                               // if the new cost to move is less than before, change the parent
                    distance_from_start[query] = tentative;                                                             // change the cost
                    cost_map[query] = tentative + calculateDistance(child, Coordinate(actual_exit[0], actual_exit[1])); // update the cost to end node
                    pathfinder.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY())));                 // update the priority queue
                }
            }
            if (!hasWallWest)
            {
                auto child = Coordinate(x - 1, y);
                int tentative = distance_from_start[parent] + 1;                                 // check the cost it takes to move to the neighboring node
                vector<int> query = {child.getX(), child.getY()};                                // create a searching query
                if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
                {
                    path[query] = parent;                                                                               // if the new cost to move is less than before, change the parent
                    distance_from_start[query] = tentative;                                                             // change the cost
                    cost_map[query] = tentative + calculateDistance(child, Coordinate(actual_exit[0], actual_exit[1])); // update the cost to end node
                    pathfinder.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY())));                 // update the priority queue
                }
            }
            auto current_coordinate = pathfinder.top(); // proceed to the next node
            x = current_coordinate.second.first;        // update X and Y values
            y = current_coordinate.second.second;
            pathfinder.pop();
        }

        return make_optional<Coordinate>(x, y); // proceed the exploration
    }

    list<Coordinate> getShortestPath()
    {

        list<Coordinate> shortest_path = {}; // intialize the path list
        vector<int> path_iterator = {x, y};  // create vector-iterator to iterate the path map
        vector<int> start_position = {0, 0}; // mark the start node

        while (path_iterator != start_position)
        {
            shortest_path.push_back(Coordinate(path_iterator[0], path_iterator[1])); // collect each node from end to start
            path_iterator = path[path_iterator];                                     // make the parent node a child and iterate further
        }
        shortest_path.push_back(Coordinate(start_position[0], start_position[1])); // add the starting node to the list
        shortest_path.reverse();                                                   // since the path is a backtrace from end to start, reverse the list
        return shortest_path;
    }
};

int main(int argc, char *argv[])
{

    int size_x, size_y;

    if (argc == 3)
    {
        size_x = atoi(argv[1]);
        size_y = atoi(argv[2]);
    }
    else
    {
        cerr << "Error: wrong arguments." << endl;
        return -1; // do nothing
    }

    DepthFirstSearch agent(size_x, size_y);

    while (true)
    {
        string s1, s2, s3, s4, s5;
        cin >> s1 >> s2 >> s3 >> s4 >> s5;

        bool isExit = (s1 != "0");
        bool hasWallSouth = (s2 != "0");
        bool hasWallNorth = (s3 != "0");
        bool hasWallEast = (s4 != "0");
        bool hasWallWest = (s5 != "0");

        auto coord = agent.move(isExit, hasWallSouth, hasWallNorth, hasWallEast, hasWallWest);

        if (coord)
        {
            cout << coord->getX() << " " << coord->getY() << endl;
        }
        else
        {
            break;
        }
    }

    list<Coordinate> path = agent.getShortestPath();

    cout << "PATH" << endl;
    for (auto &&coord : path)
    {
        cout << coord.getX() << " " << coord.getY() << endl;
    }
    cout << "END" << endl;

    return 0;
}
