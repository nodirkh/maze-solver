
/* *
 * Implementation:
 *
 * The A* algorithm is implemented on top of MinHeap (here, priority queue).
 * When visiting each node, the A* agent keeps track of the node with the smallest distance to the end node.
 * Also, the agent keeps track of the cost to make a step from parent node to child node.
 *
 * When it encounters a "cheaper" step, it efficiently (greedily) updates its MinHeap with the node of smaller cost.
 *
 * The distance from neighboring nodes to end is assumed to be the distance from the current node to the end node.
 * Once the distance becomes available, the costs are compared and updated.
 *
 * The shortest path is given as the path constructed on top of nodes that have the smallest distance metrics from
 * node X to the end node. The path is stored in a child:parent fashion. Once a better path is found, the parents are updated.
 *
 * */

#include <iostream>
#include <list>
#include <string>
#include <cstdlib>
#include <vector>
#include <queue>
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

class AStar
{
private:
  priority_queue<pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<pair<double, pair<int, int>>>> astar; // create MinHeap for node selection
  map<vector<int>, double> cost_map;                                                                                               // trace the distance cost of each node to end
  map<vector<int>, int> distance_from_start;                                                                                       // trace the cost of each node from start
  map<vector<int>, vector<int>> path;                                                                                              // trace the path in a child:parent manner
  int x, y;                                                                                                                        // track current x and y

public:
  AStar(int size_x, int size_y)
  {
    // intialize varialbes
    x = 0;
    y = 0;
    cost_map[{x, y}] = 0; // cost of start is 0
    distance_from_start[{x, y}] = 0;
  }

  optional<Coordinate> move(bool isExit, bool hasWallSouth, bool hasWallNorth, bool hasWallEast, bool hasWallWest, double distance)
  {

    if (isExit) // if exit is found, stop exploration
      return nullopt;

    vector<int> parent = {x, y};
    if (!hasWallSouth)
    {
      // check the node below

      auto child = Coordinate(x, y + 1);
      int tentative = distance_from_start[parent] + 1;                                 // calculate the tentative (g-score) cost from parent to node
      vector<int> query = {child.getX(), child.getY()};                                // create vector for checking
      if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
      {
        path[query] = parent;                                                          // if the new cost to move is less than before, change the parent
        distance_from_start[query] = tentative;                                        // update the cost to traverse from start node
        cost_map[query] = tentative + distance;                                        // update the cost to traverse to end node
        astar.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY()))); // update the priority queue
      }
    }
    if (!hasWallNorth)
    {
      auto child = Coordinate(x, y - 1);
      int tentative = distance_from_start[parent] + 1;                                 // calculate the tentative (g-score) cost from parent to node
      vector<int> query = {child.getX(), child.getY()};                                // create vector for checking
      if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
      {
        path[query] = parent;                                                          // if the new cost to move is less than before, change the parent
        distance_from_start[query] = tentative;                                        // update the cost to traverse from start node
        cost_map[query] = tentative + distance;                                        // update the cost to traverse to end node
        astar.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY()))); // update the priority queue
      }
    }
    if (!hasWallEast)
    {
      auto child = Coordinate(x + 1, y);
      int tentative = distance_from_start[parent] + 1;                                 // calculate the tentative (g-score) cost from parent to node
      vector<int> query = {child.getX(), child.getY()};                                // create vector for checking
      if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
      {
        path[query] = parent;                                                          // if the new cost to move is less than before, change the parent
        distance_from_start[query] = tentative;                                        // update the cost to traverse from start node
        cost_map[query] = tentative + distance;                                        // update the cost to traverse to end node
        astar.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY()))); // update the priority queue
      }
    }
    if (!hasWallWest)
    {
      auto child = Coordinate(x - 1, y);
      int tentative = distance_from_start[parent] + 1;                                 // calculate the tentative (g-score) cost from parent to node
      vector<int> query = {child.getX(), child.getY()};                                // create vector for checking
      if (!distance_from_start.count(query) || tentative < distance_from_start[query]) // check if the node has been examined before or tentative distance is lower
      {
        path[query] = parent;                                                          // if the new cost to move is less than before, change the parent
        distance_from_start[query] = tentative;                                        // update the cost to traverse from start node
        cost_map[query] = tentative + distance;                                        // update the cost to traverse to end node
        astar.push(make_pair(cost_map[query], make_pair(child.getX(), child.getY()))); // update the priority queue
      }
    }

    auto current_coordinate = astar.top(); // get the next node
    x = current_coordinate.second.first;   // update the X and Y values
    y = current_coordinate.second.second;
    astar.pop();

    return make_optional<Coordinate>(x, y); // continue exploration
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

  AStar agent(size_x, size_y);

  while (true)
  {
    string s1, s2, s3, s4, s5, s6;
    cin >> s1 >> s2 >> s3 >> s4 >> s5 >> s6;

    bool isExit = (s1 != "0");
    bool hasWallSouth = (s2 != "0");
    bool hasWallNorth = (s3 != "0");
    bool hasWallEast = (s4 != "0");
    bool hasWallWest = (s5 != "0");
    double distance = stof(s6);

    auto coord = agent.move(isExit, hasWallSouth, hasWallNorth, hasWallEast, hasWallWest, distance);

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
