import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Perform nearest neighbor to find an initial route
    route = nearest_neighbor(_distances)

    # Apply 2-opt local search to improve the route
    route = local_search(route, _distances)

    return route

def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Finds a route using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    unvisited = set(range(num_cities))
    route = []
    current_city = np.random.choice(list(unvisited))
    while unvisited:
        unvisited.remove(current_city)
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(current_city)
        current_city = nearest_city
    route.append(route[0])  # Close the cycle
    return tuple(route)

def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """Performs 2-opt local search to improve a route."""
    while True:
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j+1:]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    route = new_route
                    break
        else:
            break
    return route

def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i + 1) % len(route)]]
    return distance
