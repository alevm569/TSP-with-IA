import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""
    # Combine nearest neighbor and cheapest insertion heuristics
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    remaining_cities = set(range(len(_distances))) - {start_city}

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        cheapest_city = min(remaining_cities, key=lambda city: _distances[current_city][city], default=None)

        if np.random.rand() < 0.5:
            route.append(nearest_city)
        else:
            route.append(cheapest_city)

        remaining_cities.remove(route[-1])

    # Add return to starting city
    route.append(start_city)

    return tuple(route)
