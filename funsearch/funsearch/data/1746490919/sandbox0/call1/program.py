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
    """Improved version of `find_best_route_v1` with ACO."""

    # ACO parameters
    num_ants = 10
    num_iterations = 100
    pheromone_decay = 0.9
    alpha = 1
    beta = 2

    # Initialize pheromone matrix
    pheromones = np.ones_like(_distances) / len(_distances)

    # Generate initial routes
    routes = np.random.permutation(len(_distances)).reshape(-1, len(_distances))

    # ACO loop
    for _ in range(num_iterations):
        # Update pheromones
        for route in routes:
            for i in range(len(route)):
                pheromones[route[i]][route[(i + 1) % len(route)]] += 1 / calculate_route_distance(route, _distances)

        # Update routes
        routes = funsearch.aco.ant_colony_optimization(
            _distances,
            num_ants=num_ants,
            pheromones=pheromones,
            alpha=alpha,
            beta=beta,
        )

    # Return the best route
    best_route = routes[np.argmin([calculate_route_distance(route, _distances) for route in routes])]
    return best_route

