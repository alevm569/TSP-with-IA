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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function implements a hybrid heuristic that combines multiple heuristics:

    - **Nearest Neighbor:** Start from an initial city and iteratively select the nearest unvisited city.
    - **2-Opt:** Randomly swap two edges in the route and check if it improves the total distance.
    - **Tabu Search:** Use a tabu list to prevent the search from getting stuck in local minima.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(distances))
    route = [start_city]

    # Nearest Neighbor heuristic
    while len(route) < len(distances):
        current_city = route[-1]
        nearest_city = np.argmin(distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)

    # 2-Opt heuristic
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        if calculate_route_distance(route[:i] + route[j:i:-1] + route[j+1:], distances) < calculate_route_distance(route, distances):
            route = route[:i] + route[j:i:-1] + route[j+1:]

    # Tabu search
    tabu_list = []
    for _ in range(1000):
        i, j = np.random.randint(len(route), size=2)
        if (i, j) not in tabu_list and (j, i) not in tabu_list:
            distance_difference = calculate_route_distance(route[:i] + route[j:i:-1] + route[j+1:], distances) - calculate_route_distance(route, distances)
            if distance_difference < 0:
                route = route[:i] + route[j:i:-1] + route[j+1:]
                tabu_list.append((i, j))
            else:
                tabu_list.append((i, j))

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
