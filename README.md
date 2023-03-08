# Solve the maze
Project includes simulation of micromouse robot. Micromouse is an event where small robot mice solve a 16×16 maze. E-puck is used like a simulation robot.
![abyrynt](https://user-images.githubusercontent.com/100734139/223844654-1980937e-ffef-42c2-9e82-669ebf6b850c.jpg)
## Environment
* Maze: 16x16
* Cells are squares and have four corners
* Maze has a continuous outer wall on its perimeter.
* Robot can start at any location in maze.
* Robot will always face north.
*  The destination goal is a gateway to the four-cell square at the center of the maze.
* Timer included 

## Floodfill algorythm 
To solve the maze robot uses floodfill algorytm. Using knowledge of current location and orientation to:
- move to center
- build the map
- update the costs of each visited cell using
At every point in time the robot knows its position.
Everytime the robot moves to a cell, it checks if the floodfill cost allocated to the cell is valid:
-if the cost is not valid the robot will update the cost of the cell at its current position and all other neighbouring cells.
Everytime the robot visits a new cell (previously unexplored), it will sense for neighbouring walls and will update the maze map in memory.

## Modes
Robot has three modes. 
- MODE 1: user solution via keyboard
- MODE 2:  exploration of the maze
- MODE 3: speed ride
During exploration the shortest trace is written in file. Speed ride allowes to use this trace.
