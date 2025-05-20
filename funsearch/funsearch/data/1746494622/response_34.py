import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize solution with nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    current_city = start_city
    route = [current_city]

    # Generate a set of unvisited cities
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    # Iteratively find the closest unvisited city
    while unvisited_cities:
        min_distance = float('inf')
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                closest_city = city

        # Add the closest city to the route and remove it from the unvisited set
        route.append(closest_city)
        unvisited_cities.remove(closest_city)

        # Update the current city
        current_city = closest_city

    # Close the route by adding the starting city
    route.append(start_city)

    return tuple(route)
