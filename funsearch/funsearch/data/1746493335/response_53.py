import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        next_city = np.argmin(_distances[current_city][[c for c in range(len(_distances)) if c not in visited]])
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Local search to improve the route
    for i in range(100):  # Number of iterations for local search
        for j in range(len(route)):
            for k in range(j + 1, len(route)):
                new_route = route[:j] + route[k:j:-1] + route[k+1:]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < calculate_route_distance(route, _distances):
                    route = new_route

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
