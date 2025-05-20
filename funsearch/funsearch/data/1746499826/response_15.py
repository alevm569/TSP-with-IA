import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid approach combining the nearest neighbor heuristic and the 2-opt local search.
    """

    # Initialize a random route
    initial_route = np.random.permutation(np.arange(_distances.shape[0]))

    # Use nearest neighbor to find an initial solution
    current_city = initial_route[0]
    route = [current_city]
    unvisited_cities = set(initial_route[1:])

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt local search to improve the solution
    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route.copy()
            new_route[i:j+1] = route[j:i:-1]
            new_distance = calculate_route_distance(new_route, _distances)

            if new_distance < best_distance:
                best_route = new_route
                best_distance = new_distance

    return best_route
