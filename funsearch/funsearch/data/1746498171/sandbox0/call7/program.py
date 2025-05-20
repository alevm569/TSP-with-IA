"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics and if you are going to use randomness, stabilize it by setting a seed to ensure reproducibility.
PROVIDE just the python code for the new version of the function, i.e. find_best_route_vx"""
import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def calculate_route_distance(route: tuple[int, ...], distances: ndarray) -> float:
    """
    function to calculate the total distance of a given route.
    sum the distances between cities in the route
    """
    distance = sum(distances[route[i], route[i + 1]] for i in range(len(route) - 1))
    # add the distance from the last city to the first city
    distance += distances[route[-1], route[0]]
    return float(distance)


def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.
    
    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

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
    """Improved version of `find_best_route_v0`."""

    # Initialize variables
    num_cities = len(_distances)
    best_distance = float('inf')
    best_route = None

    # Generate random initial solution
    np.random.seed(42)  # Set seed for reproducibility
    initial_route = np.random.permutation(num_cities)

    # Perform local search optimization
    current_route = initial_route
    current_distance = calculate_route_distance(current_route, _distances)

    while True:
        # Generate a new candidate route by swapping two random cities
        idx1, idx2 = np.random.randint(0, num_cities, 2)
        candidate_route = current_route.copy()
        candidate_route[idx1], candidate_route[idx2] = candidate_route[idx2], candidate_route[idx1]

        # Calculate the distance of the candidate route
        candidate_distance = calculate_route_distance(candidate_route, _distances)

        # If the candidate route is better, update the current route
        if candidate_distance < current_distance:
            current_route = candidate_route
            current_distance = candidate_distance

            # Check if the current route is the best route found so far
            if current_distance < best_distance:
                best_distance = current_distance
                best_route = current_route

        # Stop if no improvement is found for a certain number of iterations
        if current_distance == best_distance:
            break

    # Return the best route found
    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

