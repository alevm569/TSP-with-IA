import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using hybrid heuristics."""

    # Initial solution using nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    current_city = start_city
    route = [current_city]

    # Generate a set of available cities
    available_cities = set(range(len(_distances)))
    available_cities.remove(start_city)

    # Hybrid heuristic: Combine 2-opt and cheapest insertion
    while available_cities:
        # 2-opt heuristic
        best_delta = float('inf')
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                delta = 2 * _distances[route[i]][route[j]] - _distances[route[(i - 1) % len(route)]][route[i]] - _distances[route[j]][route[(j + 1) % len(route)]]
                if delta < best_delta:
                    best_delta = delta
                    best_i = i
                    best_j = j

        if best_delta < 0:
            route[best_i], route[best_j] = route[best_j], route[best_i]
        else:
            # Cheapest insertion heuristic
            best_city = None
            best_distance = float('inf')
            for city in available_cities:
                distance = _distances[current_city][city]
                if distance < best_distance:
                    best_distance = distance
                    best_city = city

            route.append(best_city)
            available_cities.remove(best_city)
            current_city = best_city

    # Ensure the route returns to the starting city
    route.append(start_city)

    return tuple(route)
