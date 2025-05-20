import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid approach combining local search with a genetic algorithm
    # Initialize population of routes using nearest neighbor heuristic
    population = [nearest_neighbor(_distances) for _ in range(100)]

    # Run genetic algorithm to optimize routes
    best_route = funsearch.genetic(population, _distances, calculate_route_distance)

    # Perform local search to refine the best route
    best_route = funsearch.local_search(best_route, _distances)

    return best_route

# Helper functions for find_best_route_v2

def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Creates a route using the nearest neighbor heuristic."""
    unvisited = set(range(len(_distances)))
    route = []
    current = np.random.choice(list(unvisited))
    while unvisited:
        unvisited.remove(current)
        nearest = min(unvisited, key=lambda city: _distances[current][city])
        route.append(current)
        current = nearest
    route.append(route[0])
    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i + 1) % len(route)]]
    return distance
