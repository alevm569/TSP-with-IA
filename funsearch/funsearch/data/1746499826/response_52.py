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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a combination of local search and 2-opt heuristics
    initial_route = np.random.permutation(np.arange(len(_distances)))
    best_route = local_search(initial_route, _distances)

    # Perform 2-opt operations until convergence
    best_distance = calculate_route_distance(best_route, _distances)
    while True:
        for i in range(len(best_route)):
            for j in range(i + 2, len(best_route)):
                candidate_route = swap_edges(best_route, i, j)
                candidate_distance = calculate_route_distance(candidate_route, _distances)
                if candidate_distance < best_distance:
                    best_distance = candidate_distance
                    best_route = candidate_route

        if best_distance == calculate_route_distance(best_route, _distances):
            break

    return best_route

def local_search(route: np.ndarray, _distances: np.ndarray) -> np.ndarray:
    """Perform local search to improve a given route."""
    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        for i in range(len(best_route)):
            for j in range(i + 2, len(best_route)):
                candidate_route = swap_edges(best_route, i, j)
                candidate_distance = calculate_route_distance(candidate_route, _distances)
                if candidate_distance < best_distance:
                    best_distance = candidate_distance
                    best_route = candidate_route

        if best_distance == calculate_route_distance(best_route, _distances):
            break

    return best_route

def swap_edges(route: np.ndarray, i: int, j: int) -> np.ndarray:
    """Swap two edges in a route."""
    return np.concatenate((route[:i], route[j:i:-1], route[j+1:]))
