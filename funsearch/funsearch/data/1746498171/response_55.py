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
    Improved version of the find_best_route function using a hybrid heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function implements a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))

    # Initialize an empty list to store the route
    route = [current_city]

    # Mark the starting city as visited
    visited = np.zeros(len(matrix_distances), dtype=bool)
    visited[current_city] = True

    # Visit all other cities
    for _ in range(len(matrix_distances) - 1):
        # Find the nearest unvisited city
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if not visited[j]])

        # Find the cheapest insertion city
        cheapest_city = np.argmin([matrix_distances[j][nearest_city] for j in range(len(matrix_distances)) if not visited[j]])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Mark the nearest city as visited
        visited[nearest_city] = True

        # Set the current city to the nearest city
        current_city = nearest_city

    # Add the starting city back to the route
    route.append(route[0])

    return tuple(route)
