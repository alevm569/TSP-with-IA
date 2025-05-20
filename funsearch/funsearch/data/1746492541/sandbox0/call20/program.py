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
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Initialize ACO parameters
    num_ants = 10
    num_iterations = 100
    alpha = 1  # Pheromone influence
    beta = 2  # Heuristic influence
    rho = 0.1  # Pheromone evaporation rate

    # Initialize pheromone matrix
    pheromones = np.ones_like(_distances)

    # Initialize best route
    best_route = None
    best_distance = float('inf')

    # Run ACO algorithm
    for iteration in range(num_iterations):
        # Send ants to explore
        routes = []
        for ant in range(num_ants):
            route = acs(_distances, pheromones, alpha, beta)
            routes.append(route)

        # Update pheromones
        for route in routes:
            distance = calculate_route_distance(route, _distances)
            for i in range(len(route)):
                pheromones[route[i]][route[(i + 1) % len(route)]] += 1 / distance

        # Update best route
        for route in routes:
            distance = calculate_route_distance(route, _distances)
            if distance < best_distance:
                best_distance = distance
                best_route = route

    return best_route

def acs(_distances: np.ndarray, pheromones: np.ndarray, alpha: float, beta: float) -> tuple[int, ...]:
    # Implement ACO heuristic here
    pass

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    # Calculate total distance of a route
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

