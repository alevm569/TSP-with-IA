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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new version of the find_best_route function here.
    # You can use any of the strategies or heuristics mentioned in the problem description.

    # Example of a simple heuristic:
    # 1. Start at the first city.
    # 2. Find the city with the minimum distance to the current city.
    # 3. Add the next city to the route.
    # 4. Repeat until all cities have been visited.
    # 5. Return to the starting city.

    # Initialize the route with the first city
    route = [0]

    # Mark the first city as visited
    visited = [False] * len(_distances)
    visited[0] = True

    # Iterate until all cities have been visited
    while len(route) < len(_distances):
        # Find the city with the minimum distance to the current city
        min_distance = math.inf
        next_city = None
        for i in range(len(_distances)):
            if not visited[i] and _distances[route[-1]][i] < min_distance:
                min_distance = _distances[route[-1]][i]
                next_city = i

        # Add the next city to the route
        route.append(next_city)

        # Mark the next city as visited
        visited[next_city] = True

    # Return to the starting city
    route.append(0)

    return tuple(route)
