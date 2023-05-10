CC=g++
CFLAGS=-std=c++17 -O3

astar: astar.cpp
	$(CC) $(CFLAGS) -o astar astar.cpp
bfs: bfs.cpp
	$(CC) $(CFLAGS) -o bfs bfs.cpp
dfs: dfs.cpp
	$(CC) $(CFLAGS) -o dfs dfs.cpp

all: astar bfs dfs
clean: 
	rm -f astar bfs dfs
.PHONY: all
