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
    Improved version of find_best_route_v1 using a hybrid heuristic.

    Hybrid heuristic combines the nearest neighbor and cheapest insertion heuristics.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize an empty route.
    route = []

    # Start from the first city.
    current_city = 0

    # Mark the first city as visited.
    visited = np.zeros(distances.shape[0], dtype=bool)
    visited[current_city] = True

    # Generate a list of unvisited cities.
    unvisited = np.where(~visited)[0]

    # Perform nearest neighbor and cheapest insertion heuristics iteratively.
    for _ in range(distances.shape[0]):
        # Find the nearest unvisited city.
        nearest_city = unvisited[np.argmin(distances[current_city][unvisited])]

        # Find the cheapest city to insert next.
        cheapest_city = unvisited[np.argmin(distances[nearest_city][unvisited])]

        # Add the cheapest city to the route.
        route.append(cheapest_city)

        # Mark the cheapest city as visited.
        visited[cheapest_city] = True

        # Remove the cheapest city from the list of unvisited cities.
        unvisited = np.delete(unvisited, np.where(unvisited == cheapest_city)[0][0])

        # Update the current city.
        current_city = cheapest_city

    # Close the route by adding the starting city.
    route.append(route[0])

    return tuple(route)
