# PathPlanning
ROS2 Humble Pathplanning Stack

Planner Sources in `modules` folder of each packages.

Available Planners
- A* Planner
- Model Predictive Planner : MPC
- Dijkstra Planner (under construction)
- D* Planner (under construction)
- RRT* Planner (under construction)
- Pure Pursuit (under construction)

## Global Planner
### Dijkstra
 Dijkstra's algorithm is a fundamental graph search algorithm used to find the shortest path from a single source node to all other nodes in a weighted graph. It operates by iteratively selecting the node with the shortest tentative distance from the source and updating the distances to its neighbors. Initially, all nodes have an infinite distance from the source except the source itself, which has a distance of zero. As the algorithm progresses, it continually expands the set of visited nodes while ensuring that the shortest path to each node is found. Dijkstra's algorithm is guaranteed to produce correct results for graphs with non-negative edge weights and is often employed in routing protocols, network analysis, and other optimization problems.

### A* Planner
 The A* algorithm efficiently finds the shortest path between two points on a graph by evaluating nodes based on their tentative distance from the start and estimated distance to the goal, using a priority queue. It iteratively selects the node with the lowest total estimated distance and updates distances to its neighbors if a shorter path is found. A* guarantees completeness and optimality under certain conditions, making it widely used in pathfinding applications, though its performance relies on the choice of heuristic function.

 ### D* Planner
D* (pronounced "D-star") is an incremental search algorithm primarily used for path planning in dynamic and partially known environments, where the robot or agent must adapt its path as new information becomes available. Unlike traditional pathfinding algorithms, D* starts with an initial path and incrementally updates it as changes occur in the environment, such as obstacles being added or removed. It operates by maintaining a cost-to-go map, which represents the estimated cost from each cell to the goal, and propagating updates backwards from the goal to the start whenever changes occur. This allows D* to quickly re-plan and adjust the path based on new information while minimizing computational overhead. D* is particularly well-suited for scenarios where real-time re-planning is required, such as robotics and autonomous systems navigating dynamic environments.

### RRT* Planner
RRT* (Rapidly-exploring Random Tree Star) is a sampling-based motion planning algorithm commonly used in robotics and autonomous systems to find feasible paths in high-dimensional configuration spaces. It incrementally builds a tree rooted at the initial configuration by randomly sampling the space and expanding the tree towards unexplored regions. RRT* improves upon the original RRT algorithm by optimizing the tree structure to minimize path length, making it more efficient and capable of finding higher-quality paths. It achieves this optimization by re-wiring the tree connections based on cost-to-come information from nearby nodes. RRT* is well-suited for complex, high-dimensional spaces and dynamic environments where traditional search-based planning methods may struggle, providing a flexible and efficient approach to motion planning tasks.


## Local Planner
### Model Predictive Planner
 Model Predictive Control (MPC) is an advanced control strategy used in various fields such as robotics, automotive, and process control. Unlike traditional control methods that compute control actions based on a fixed model and past measurements, MPC predicts future system behavior using a dynamic model and optimizes control actions over a finite time horizon. At each time step, MPC solves an optimization problem to find the control sequence that minimizes a cost function while satisfying system constraints. By repeatedly updating predictions and control actions, MPC adapts to changes in the system and disturbances, enabling precise and robust control in complex and nonlinear systems.

 ### PurePursuit
 Pure Pursuit is a simple and effective path tracking algorithm commonly used in robotics and autonomous vehicle navigation. It involves following a predefined path by steering the vehicle to a point ahead of it on the path, known as the lookahead point. The algorithm calculates the lookahead point based on the vehicle's current position, velocity, and turning radius, ensuring smooth and stable tracking. By continuously updating the lookahead point as the vehicle moves along the path, Pure Pursuit enables precise and agile trajectory tracking, even in dynamic environments. Its simplicity, efficiency, and effectiveness make it a popular choice for real-time motion control tasks, particularly in applications where rapid response and smooth trajectory following are crucial.
