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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    We use a hybrid heuristic that combines two strategies:

    - **Nearest Neighbor:** Start from an arbitrary city and iteratively choose the closest unvisited city.
    - **Tabu Search:** Employ a tabu search algorithm to explore nearby solutions, avoiding repeating visited cities.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route and visited set
    num_cities = len(matrix_distances)
    route = np.zeros(num_cities, dtype=int)
    visited = set()

    # Start from an arbitrary city
    start_city = np.random.randint(num_cities)
    route[0] = start_city
    visited.add(start_city)

    # Implement the hybrid heuristic
    for i in range(1, num_cities):
        # Nearest Neighbor strategy
        current_city = route[i - 1]
        closest_city = None
        min_distance = np.inf

        for city in range(num_cities):
            if city not in visited:
                distance = matrix_distances[current_city][city]
                if distance < min_distance:
                    min_distance = distance
                    closest_city = city

        # Tabu search strategy
        tabu_list = set()
        iterations = 100

        while True:
            candidate_city = np.random.choice(num_cities)
            if candidate_city not in visited and candidate_city not in tabu_list:
                break

            tabu_list.add(candidate_city)

        route[i] = candidate_city
        visited.add(candidate_city)

        if len(visited) == num_cities:
            route = np.append(route, start_city)
            break

    return route
