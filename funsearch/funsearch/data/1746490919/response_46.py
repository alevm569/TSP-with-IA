import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v0` using a hybrid heuristic.

    Hybrid heuristic combines two strategies:
    - Nearest neighbor: Starts from a random city and iteratively finds the nearest unvisited city.
    - Cheapest insertion: Finds the cheapest city to insert into the current route at each step.

    Returns:
    A permutation of cities that minimizes the total route distance.
    """

    # Initialize random starting city
    start_city = np.random.randint(len(distances))

    # Initialize route with starting city
    route = [start_city]

    # Mark starting city as visited
    visited = np.zeros(len(distances), dtype=bool)
    visited[start_city] = True

    # Iterate until all cities are visited
    while not np.all(visited):
        # Nearest neighbor strategy
        nearest_city = -1
        min_distance = math.inf
        for i in range(len(distances)):
            if not visited[i]:
                distance = distances[route[-1]][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Cheapest insertion strategy
        cheapest_city = -1
        min_insertion_distance = math.inf
        for i in range(len(distances)):
            if not visited[i]:
                insertion_distance = distances[route[-1]][i] + distances[i][route[0]]
                if insertion_distance < min_insertion_distance:
                    cheapest_city = i
                    min_insertion_distance = insertion_distance

        # Add the nearest city to the route
        route.append(nearest_city)

        # Mark the nearest city as visited
        visited[nearest_city] = True

    # Add the starting city back to the route
    route.append(start_city)

    return tuple(route)
