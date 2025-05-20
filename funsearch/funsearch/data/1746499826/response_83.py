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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Nearest neighbor: Select the nearest unvisited city at each step.
    - Local search: Iterate through the route and swap two adjacent cities to find a shorter distance.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))

    # Initialize an empty route
    route = []

    # Mark the starting city as visited
    visited = np.zeros(len(matrix_distances), dtype=bool)
    visited[current_city] = True

    # Perform nearest neighbor search
    for _ in range(len(matrix_distances)):
        # Find the nearest unvisited city
        nearest_city = -1
        min_distance = math.inf
        for i in range(len(matrix_distances)):
            if not visited[i]:
                distance = matrix_distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route
        route.append(nearest_city)

        # Mark the nearest city as visited
        visited[nearest_city] = True

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
