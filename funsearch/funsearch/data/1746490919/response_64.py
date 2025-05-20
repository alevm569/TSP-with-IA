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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic combining the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Create an empty route
    route = [start_city]

    # Create a list of available cities
    available_cities = list(range(len(matrix_distances)))
    available_cities.remove(start_city)

    # Perform nearest neighbor and cheapest insertion iteratively
    while available_cities:
        # Get the last city in the route
        current_city = route[-1]

        # Find the nearest city not already in the route
        nearest_city = available_cities[np.argmin([matrix_distances[current_city][c] for c in available_cities])]

        # Find the cheapest city to insert next
        cheapest_city = available_cities[np.argmin([matrix_distances[current_city][c] for c in available_cities if c != nearest_city])]

        # Add the cheapest city to the route
        route.append(cheapest_city)
        available_cities.remove(cheapest_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
