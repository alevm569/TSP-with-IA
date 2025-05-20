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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with a random city
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    # Generate the remaining cities in the route
    remaining_cities = set(range(len(_distances))) - {current_city}

    # Iterate until all cities have been visited
    while remaining_cities:
        # Select the next city using a combination of nearest neighbor and cheapest insertion
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        cheapest_city = min(remaining_cities, key=lambda city: _distances[current_city][city] + _distances[city][nearest_city])

        # Add the cheapest city to the route
        route.append(cheapest_city)
        remaining_cities.remove(cheapest_city)

        # Update the current city
        current_city = cheapest_city

    # Close the route by adding the starting city
    route.append(route[0])

    return tuple(route)
