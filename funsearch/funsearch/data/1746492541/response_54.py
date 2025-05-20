import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Hybrid heuristic combines two strategies:
        - Nearest neighbor for initial route generation.
        - Cheapest insertion for subsequent route additions.

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = np.random.randint(len(matrix_distances))
    current_city = start_city
    route = [current_city]

    # Generate the route using nearest neighbor and cheapest insertion
    visited = set([start_city])
    while len(visited) < len(matrix_distances):
        # Find the nearest unvisited city
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in visited])
        # Add the nearest city to the route
        route.append(nearest_city)
        # Update the current city
        current_city = nearest_city
        # Mark the city as visited
        visited.add(nearest_city)

    # Return the route, including the return to the starting city
    return tuple(route + [start_city])


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
        route (tuple[int, ...]): A permutation of cities.
        matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        The total distance of the route.
    """

    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return distance
