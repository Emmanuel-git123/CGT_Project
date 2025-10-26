# Disaster Management System
To Develop and validate a graph-based system in C++ that optimizes emergency response in an area containing Houses, Ambulance Site, and Hospitals, ensuring both path safety (Checking Water Level) and Efficient Path (Distance minimization).
## Representation:
- Houses, Ambulance station, Hospital and intersection roads are taken as nodes.
- the roads connecting the above are taken as edges.
- Determines the houses which were affected by the floods and remove them from the graph.
- The corresponding edges also removed.
## Features:
- Explore all the paths and finds whether there exist a path for the houses from Hospitals for any medical support.
- Connects Ambulance stations to houses and houses to hospitals for immediate emergency situations.
- A shortest path connects all the houses for the food supply to all people affect by Disaster.
## How to use:
- Open the c++ code in any application like VS Code.
- Compile the file in terminal and run the file.
- It automatically generates a Disaster and the weight of the edges (distance between two nodes).
- The nodes which are lower than the level of flood water were affected by disaster and the people from the affected nodes were moved to the nearby nodes.
## Algorithms used:
- BFS Algorithm: It is used to find the whether a path is available for houses from any hospital for any medical support during the disaster.
- Dijkstra's Algorithm: It is used to find the shortest path from Ambulance station to an given emergency house and after from their to nearby Hospital.
- Prim's Algorithm (Minimal spanning tree): It is used to connect all the houses along a tree, such that to supply food and first aid kits to all the people in a short amount of time.
## Project Structure:
- */project.cpp/* -> this file contains c++ code of working Disaster Management System Simulation.