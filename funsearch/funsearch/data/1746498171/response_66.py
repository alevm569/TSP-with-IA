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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function combines two heuristics:
    - Nearest neighbor: Starts from the first city and iteratively selects the closest unvisited city.
    - 2-opt: Iteratively swaps two edges in the route to find a shorter path.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route using nearest neighbor
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(len(distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        closest_city = min(unvisited_cities, key=lambda c: distances[current_city][c])
        route.append(closest_city)
        unvisited_cities.remove(closest_city)

    # Perform 2-opt local search to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = distances[route[i]][route[j]]
            distance_reversed = distances[route[i]][route[(j - 1)]] + distances[route[(j - 1)]][route[j]] - distance_original
            if distance_reversed < distance_original:
                route[i+1:j] = route[i+1:j][::-1]

    return tuple(route)
