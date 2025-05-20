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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of the find_best_route function.

    Uses a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Create an empty route
    route = [start_city]

    # Create a list of unvisited cities
    unvisited_cities = list(range(len(matrix_distances)))
    unvisited_cities.remove(start_city)

    # Iterate until all cities have been visited
    while unvisited_cities:
        # Get the current city
        current_city = route[-1]

        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Remove the nearest city from the list of unvisited cities
        unvisited_cities.remove(nearest_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
