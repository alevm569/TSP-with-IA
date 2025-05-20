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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic combining
    nearest neighbor and 2-opt.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial route using nearest neighbor
    current_city = np.random.randint(len(distances))
    route = [current_city]

    while len(route) < len(distances):
        # Find the closest unvisited city
        closest_city = np.argmin(distances[current_city][~np.isin(np.arange(len(distances)), route)])
        route.append(closest_city)
        current_city = closest_city

    # Perform 2-opt optimization to improve the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                # Calculate the distance of the original route
                original_distance = distances[route[i]][route[(i + 1) % len(route)]] + distances[route[j]][route[(j + 1) % len(route)]]

                # Calculate the distance of the modified route
                modified_distance = distances[route[i]][route[j]] + distances[route[(i + 1) % len(route)]][route[(j + 1) % len(route)]]

                # If the modified route is shorter, swap the cities
                if modified_distance < original_distance:
                    route[i+1], route[j] = route[j], route[i+1]

    return tuple(route)
