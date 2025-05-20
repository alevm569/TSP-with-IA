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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Generate a random starting city
    np.random.seed(0)
    start_city = np.random.randint(len(_distances))

    # Initialize an empty route
    route = [start_city]

    # Create a set of visited cities
    visited = {start_city}

    # While all cities have not been visited
    while len(visited) < len(_distances):
        # Find the closest unvisited city to the last city in the route
        current_city = route[-1]
        min_distance = np.inf
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Add the next city to the route and mark it as visited
        route.append(next_city)
        visited.add(next_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
