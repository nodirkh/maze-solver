/* *
 * Implementation:
 *
 * Here, the original BFS algorithm is used. At the each "move" cycle, the agent checks the cell's surroundings and
 * adds all the non-wall neighbors into the queue data structure.
 * 
 *                  N
 * 
 *              W   A   E
 * 
 *                  S
 * 
 * (A is agent)
 * 
 * At each step, 4 cells are uncovered.
 * 
 * At each move, the queue front is accessed to explore the maze in layers. 
 * Once the agent finds the exit node, it backtraces the path to the origin node to construct the
 * shortest path.
 * 
 * The path is constructed by using a map in a child:parent fashion. It can backtrace from {end.X, end.Y} to {start.X, start.Y}
 * efficiently.
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

class BreadthFirstSearch
{
private:
  queue<Coordinate> bfs;              // queue to store the layer of adjacent points
  set<pair<int, int>> visited;        // check if the node is visited to avoid loops
  map<vector<int>, vector<int>> path; // trace the path in a child:parent manner
  int x;                              // track current x
  int y;                              // track current y

public:
  BreadthFirstSearch(int size_x, int size_y)
  {

    x = 0; // initialize the starting coordinate X
    y = 0; // initialize the starting coordinate Y
  }

  optional<Coordinate> move(bool isExit, bool hasWallSouth, bool hasWallNorth, bool hasWallEast, bool hasWallWest)
  {

    if (isExit)
      return nullopt; // if exit is found, return empty optional

    vector<int> parent = {x, y};

    if (!hasWallSouth)
    {
      // check the wall below

      auto child = Coordinate(x, y + 1); // create a child coordinate for getX and getY functions

      if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been seen
      {
        bfs.push(child);                                                 // if the node is new to discover, push it to the queue
        visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark the node as seen
        path[{child.getX(), child.getY()}] = parent;                     // mark the origin of the node
      }
    }
    if (!hasWallNorth)
    {
      // check the wall above

      auto child = Coordinate(x, y - 1); // create a child coordinate for getX and getY functions

      if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been seen
      {
        bfs.push(child);                                                 // if the node is new to discover, push it to the queue
        visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark the node as seen
        path[{child.getX(), child.getY()}] = parent;                     // mark the origin of the node
      }
    }
    if (!hasWallEast)
    {
      // check the wall on the right

      auto child = Coordinate(x + 1, y); // create a child coordinate for getX and getY functions

      if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been seen
      {
        bfs.push(child);                                                 // if the node is new to discover, push it to the queue
        visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark the node as seen
        path[{child.getX(), child.getY()}] = parent;                     // mark the origin of the node
      }
    }
    if (!hasWallWest)
    {
      // check the wall on the left

      auto child = Coordinate(x - 1, y); // create a child coordinate for getX and getY functions

      if (visited.find(make_pair<int, int>(child.getX(), child.getY())) == visited.end()) // check if the node has already been seen
      {
        bfs.push(child);                                                 // if the node is new to discover, push it to the queue
        visited.insert(make_pair<int, int>(child.getX(), child.getY())); // mark the node as seen
        path[{child.getX(), child.getY()}] = parent;                     // mark the origin of the node
      }
    }

    auto current_coordinate = bfs.front(); // get the adjacent node from the queue
    x = current_coordinate.getX();
    y = current_coordinate.getY();
    bfs.pop(); // pop the node

    return make_optional<Coordinate>(x, y); // return node to explore furhter
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

  BreadthFirstSearch agent(size_x, size_y);

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
