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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    np.random.seed(0)  # Set seed for reproducibility

    # Initialize a random permutation of cities
    cities = np.random.permutation(len(_distances))

    # Perform local search to improve the route
    for _ in range(100):
        i, j = np.random.randint(0, len(_distances), size=2)
        cities = local_search(cities, _distances, i, j)

    return cities


def local_search(cities: np.ndarray, _distances: np.ndarray, i: int, j: int) -> np.ndarray:
    """Perform local search to improve a route by swapping two cities."""
    old_distance = calculate_route_distance(cities, _distances)

    # Swap the two cities
    cities[i], cities[j] = cities[j], cities[i]

    new_distance = calculate_route_distance(cities, _distances)

    if new_distance < old_distance:
        return cities
    else:
        # If the new route is not better, swap the cities back
        cities[i], cities[j] = cities[j], cities[i]
        return cities


def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i + 1) % len(route)]]
    return distance
