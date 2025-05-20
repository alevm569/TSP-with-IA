import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(0)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with additional heuristics."""

    # Use a combination of local search and 2-opt heuristics
    initial_route = np.random.permutation(len(_distances))
    best_route = local_search(initial_route, _distances)
    best_route = two_opt(best_route, _distances)

    return best_route

# Local search heuristic
def local_search(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, distances)

    while True:
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = swap(route, i, j)
                new_distance = calculate_route_distance(new_route, distances)

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        if best_distance == calculate_route_distance(route, distances):
            break
        else:
            route = best_route.copy()

    return best_route

# 2-opt heuristic
def two_opt(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, distances)

    while True:
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                new_route = swap(route, i, j)
                new_distance = calculate_route_distance(new_route, distances)

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        if best_distance == calculate_route_distance(route, distances):
            break
        else:
            route = best_route.copy()

    return best_route

# Helper functions
def swap(route: np.ndarray, i: int, j: int) -> np.ndarray:
    route[i], route[j] = route[j], route[i]
    return route

def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
