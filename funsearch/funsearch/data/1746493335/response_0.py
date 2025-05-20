import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Perform nearest neighbor to get an initial solution.
    start_city = 0
    route = [start_city]
    remaining_cities = list(range(1, len(_distances)))
    while remaining_cities:
        current_city = route[-1]
        closest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(closest_city)
        remaining_cities.remove(closest_city)

    # Perform local search to refine the solution.
    for _ in range(100):  # Perform 100 iterations of local search
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                # Swap two cities in the route.
                route[i], route[j] = route[j], route[i]

                # Calculate the total distance of the new route.
                total_distance = calculate_route_distance(route, _distances)

                # If the new route is better, keep it.
                if total_distance < calculate_route_distance(route, _distances):
                    break
                else:
                    # Otherwise, revert the swap.
                    route[i], route[j] = route[j], route[i]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
