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
    """Improved version of `find_best_route_v0`."""

    # Use a combination of the nearest neighbor and 2-opt heuristics.
    # Nearest neighbor: Generate an initial route by iteratively selecting the city that is closest to the current city.
    # 2-opt: Iteratively swap two cities in the route to find a shorter route.

    # Create an initial route using nearest neighbor.
    current_city = 0
    route = [current_city]
    while len(route) < len(_distances):
        closest_city = np.argmin(_distances[current_city])
        if closest_city not in route:
            route.append(closest_city)
            current_city = closest_city

    # Perform 2-opt swaps to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = _distances[route[i]][route[j]] + _distances[route[j]][route[(j + 1) % len(route)]] + _distances[route[i]][route[(i + 1) % len(route)]]
            distance_reversed = _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[i]] + _distances[route[(j + 1) % len(route)]][route[(i + 1) % len(route)]]
            if distance_reversed < distance_original:
                route[i], route[j] = route[j], route[i]

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    assert len(set(route)) == len(_distances)
    assert route[0] == route[-1]

    return tuple(route)


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use ACO (ant colony optimization) to find a good route.
    # ACO is a metaheuristic algorithm that uses a colony of ants to explore the solution space.
    # Each ant builds a partial route, and the best routes are selected to become new ants.

    # Create an ACO object.
    aco = funsearch.aco.ACO(distance_matrix=_distances)

    # Run the ACO algorithm to find the best route.
    best_route = aco.run()

    # Return the best route found.
    return best_route
