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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Use a hybrid approach combining two heuristics:
    # 1. Nearest neighbor: Start from an arbitrary city and iteratively find the nearest unvisited city.
    # 2. 2-opt: Randomly swap two cities in the route and check if it improves the total distance.

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Build the route using the nearest neighbor heuristic
    route = [start_city]
    unvisited_cities = set(range(len(matrix_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Randomly swap two cities in the route using the 2-opt heuristic
    np.random.seed(0)  # Set a seed for reproducibility
    for _ in range(100):  # Perform 100 iterations of the 2-opt heuristic
        i, j = np.random.randint(0, len(route), size=2)
        if calculate_route_distance(route, matrix_distances) > calculate_route_distance(route[i:j+1][::-1] + route[:i] + route[j+1:], matrix_distances):
            route = route[i:j+1][::-1] + route[:i] + route[j+1:]

    return tuple(route)


def calculate_route_distance(route: tuple[int], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
