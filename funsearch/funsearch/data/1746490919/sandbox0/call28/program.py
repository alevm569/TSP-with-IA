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

    # Apply k-opt heuristic to generate initial route
    initial_route = k_opt(_distances)

    # Perform local search to refine the route
    best_route = local_search(initial_route, _distances)

    return best_route

def k_opt(distances: np.ndarray) -> tuple[int, ...]:
    """Generates an initial route using the k-opt heuristic."""

    # Create a random permutation of cities
    cities = np.random.permutation(np.arange(len(distances)))

    # Apply k-opt with k=2 iteratively until no improvement is found
    improved = True
    while improved:
        improved = False
        for k in range(2, len(cities)):
            for i in range(len(cities) - k):
                # Swap two sub-sequences of length k
                new_route = cities[:i] + cities[i+k:] + cities[i:i+k]

                # Calculate the total distance of the new route
                new_distance = calculate_route_distance(new_route, distances)

                # If the new route is shorter, update the current route
                if new_distance < calculate_route_distance(cities, distances):
                    cities = new_route
                    improved = True

    return cities

def local_search(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    """Refines the route using the local search heuristic."""

    # Iterate until no improvement is found
    improved = True
    while improved:
        improved = False
        for i in range(len(route)):
            for j in range(i+1, len(route)):
                # Swap two cities in the route
                new_route = route[:i] + (route[j],) + route[i+1:j] + (route[i],) + route[j+1:]

                # Calculate the total distance of the new route
                new_distance = calculate_route_distance(new_route, distances)

                # If the new route is shorter, update the current route
                if new_distance < calculate_route_distance(route, distances):
                    route = new_route
                    improved = True

