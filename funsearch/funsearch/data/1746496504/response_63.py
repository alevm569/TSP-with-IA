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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - Nearest neighbor: Start from an initial city and iteratively find the city with the minimum distance.
        - 2-opt: Randomly choose two edges and reverse their order.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(distances))

    # Create a list of unvisited cities
    unvisited_cities = list(range(len(distances)))
    unvisited_cities.remove(start_city)

    # Initialize the route
    route = [start_city]

    # Iterate until all cities have been visited
    while unvisited_cities:
        # Get the current city
        current_city = route[-1]

        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Close the route by returning to the starting city
    route.append(start_city)

    # Perform 2-opt optimization
    for _ in range(100):
        # Randomly select two edges
        i, j = np.random.randint(len(route), size=2)
        route[i:j] = route[i:j][::-1]

    return tuple(route)
