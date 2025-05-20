import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using hybrid heuristics."""

    # Perform nearest neighbor search to find an initial solution
    start_city = np.random.randint(len(_distances))
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        # Find the city with the shortest distance to the current city that is not already in the route
        next_city = np.argmin([_distances[current_city][j] for j in range(len(_distances)) if j not in route])
        route.append(next_city)
        current_city = next_city

    # Close the route by adding the starting city to the end
    route.append(start_city)

    # Perform local search to improve the solution
    for i in range(100):
        # Randomly swap two cities in the route
        a, b = np.random.randint(len(route), size=2)
        route[a], route[b] = route[b], route[a]

        # Calculate the distance of the new route
        distance = calculate_route_distance(route, _distances)

        # If the new route is better, keep it
        if distance < calculate_route_distance(route, _distances):
            continue

        # Otherwise, restore the original route
        route[a], route[b] = route[b], route[a]

    return tuple(route)
