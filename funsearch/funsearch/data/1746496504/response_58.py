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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic approach.

    This function combines two heuristics:
        - **Nearest neighbor:** Starts at a random city and iteratively selects the closest unvisited city.
        - **2-opt:** Randomly swaps two edges in the route and checks if it improves the total distance.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize random starting city
    start_city = np.random.randint(len(distances))

    # Nearest neighbor heuristic
    route = [start_city]
    visited = set([start_city])
    for _ in range(len(distances) - 1):
        current_city = route[-1]
        closest_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if j not in visited])
        route.append(closest_city)
        visited.add(closest_city)

    # 2-opt heuristic
    for _ in range(10):  # Number of iterations for 2-opt heuristic
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

        # Check if the new route is shorter
        if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
            pass  # Keep the new route
        else:
            route[i], route[j] = route[j], route[i]  # Restore the original route

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
