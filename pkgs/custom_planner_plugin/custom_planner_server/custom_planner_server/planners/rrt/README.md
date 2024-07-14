# Rapidly-exploring Random Tree (RRT) Implementation

This repository contains a custom implementation of the Rapidly-exploring Random Tree (RRT) algorithm. The RRT module can function independently, but it is specifically designed for integration with the `custom_planner_pkg`. It works in conjunction with the `custom_planner_server` class to generate paths for the navigation2 stack.

RRT is a probabilistic, sampling-based algorithm used for path planning in robotics and autonomous vehicles, among other applications. It randomly samples the search space and builds a tree towards the samples, allowing it to quickly explore large spaces.

## Algorithm Implementation

The RRT algorithm is implemented as follows:

1. **Initialization:** The RRT class is initialized with a grid representing the environment, an obstacle threshold, a maximum number of iterations, and a step size. The grid is a 2D list where 0 indicates free space and values above the obstacle threshold indicate obstacles. The nodes list is initialized to store all nodes in the RRT tree.

2. **Path Finding:** The `find_path` method generates a path from start to end using the RRT algorithm. It initializes the start and end nodes and adds the start node to the nodes list. Then it enters a loop for a maximum number of iterations.

3. **Random Node Generation:** In each iteration, a random node is generated using the `random_node` method. This method has a 10% chance of returning the end node, a 10% chance of returning a low weight vertex (a node with a low grid value), and an 80% chance of returning a completely random node in the grid.

4. **Nearest Node Finding:** The `nearest_node` method is used to find the nearest node in the RRT tree to the randomly generated node.

5. **Steering:** The `steer` method is used to create a new node in the direction from the nearest node to the random node with a maximum distance of step size.

6. **Node Validation:** The `is_valid_node` method checks if the new node is within the grid and not in an obstacle.

7. **Node Addition:** If the new node is valid, it is added to the nodes list and its parent is set to the nearest node.

8. **Goal Checking:** The `is_goal_reached` method checks if the new node is the end node. If it is, the `construct_path` method is used to construct the path from start to end and calculate the total cost.

9. **Path Construction:** The `construct_path` method constructs the path from the end node to the start node by following the parent nodes and calculates the total cost as the sum of the Euclidean distances between each node and its parent.

10. **Termination:** If the goal is not reached within the maximum number of iterations, the method returns None.

## Class UML Diagram

![UML](./readme_resources/RRT%20UML.png)

## Time and Space Complexity

The RRT algorithm has a time and space complexity of O(n), where n is the number of iterations. Each iteration involves generating a random node, finding the nearest node, and potentially adding a new node. These complexities can vary based on the implementation and the environment.

## Testing

Testing was conducted through the creation of the `tests` module, which contains a `test_rrt.py` integration testing class. This class uses the expected parameters extracted from the navigation stack, including a 2D occupancy grid read from the `test_grids/tb3_world_costmap.csv` file. This file contains all the cell values for the entire cost map from `tb3_world`. Several start and end poses were set to test the reliability of the algorithm in generating paths for this cost map.

To run the tests, use the following command:

```shell
python3 -m rrt.tests.test_rrt 
```

## Test Results

### Test Output

![Test Output](./readme_resources/rrt_tests%20output.png)

### Path Outputs from Tests

| Test 1 | Test 2 |
|--------|--------|
| ![Test 1](./tests/tests_data/test%20paths/test1.png) | ![Test 2](./tests/tests_data/test%20paths/test2.png) |

| Test 3 | Test 4 |
|--------|--------|
| ![Test 3](./tests/tests_data/test%20paths/test3.png) | ![Test 4](./tests/tests_data/test%20paths/test4.png) |

**Test 5**

![Test 5](./tests/tests_data/test%20paths/test5.png)

## Test Data

| Test Costs | Test Times |
|------------|------------|
| ![Test Costs](./readme_resources/Test%20Costs.png) | ![Test Times](./readme_resources/Test%20Times.png) |

