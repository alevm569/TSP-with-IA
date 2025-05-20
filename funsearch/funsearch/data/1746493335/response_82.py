```python
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


def find_best_route_v0(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.
    
    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v0`."""

    # Use nearest neighbor heuristic to initialize route
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply 2-opt heuristic to improve route quality
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = _distances[route[i]][route[j]]
            route[i+1:j] = reversed(route[i+1:j])
            distance_after = _distances[route[i]][route[j]]
            if distance_after < distance_before:
                break

    return tuple(route)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    """Improved version of `find_best_route_v0`."""

    # Use nearest neighbor heuristic to initialize route
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply 2-opt heuristic to improve route quality
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = _distances[route[i]][route[j]]
            route[i+1:j] = reversed(route[i+1:j])
            distance_after = _distances[route[i]][route[j]]
            if distance_after < distance_before:
                break

    return tuple(route)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a combination of nearest neighbor and 2-opt heuristics
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

        # Apply 2-opt heuristic to improve route quality
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_before = _distances[route[i]][route[j]]
                route[i+1:j] = reversed(route[i+1:j])
                distance_after = _distances[route[i]][route[j]]
                if distance_after < distance_before:
                    break

    return tuple(route)


# Set the random seed for reproducibility
np.random.seed(42)

# Run the TSP solver with the improved heuristics
best_route = find_best_route_v2(matrix_distances)

# Print the best route
print("Best Route:", best_route)
