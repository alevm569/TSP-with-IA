import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize a random starting city
    np.random.seed(42)  # Set a seed for reproducibility
    start_city = np.random.randint(len(_distances))

    # Create an empty list to store the route
    route = [start_city]

    # Mark the starting city as visited
    visited = set([start_city])

    # Iterate until all cities are visited
    while len(visited) < len(_distances):
        # Find the nearest unvisited city
        nearest_city = -1
        min_distance = float('inf')
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[route[-1]][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Find the cheapest city to insert after the last visited city
        cheapest_city = -1
        min_insertion_distance = float('inf')
        for i in range(len(_distances)):
            if i not in visited:
                insertion_distance = _distances[route[-1]][i] + _distances[i][nearest_city]
                if insertion_distance < min_insertion_distance:
                    cheapest_city = i
                    min_insertion_distance = insertion_distance

        # Add the cheapest city to the route and mark it as visited
        route.append(cheapest_city)
        visited.add(cheapest_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
