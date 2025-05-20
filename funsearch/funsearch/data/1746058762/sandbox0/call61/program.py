"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics
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
    return int(distance)


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

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v1`."""

    # Use the ant colony optimization algorithm (ACO)
    num_cities = len(_distances)
    num_ants = 10
    alpha = 1  # Pheromone strength
    beta = 2  # Heuristic strength
    rho = 0.1  # Pheromone evaporation rate

    # Initialize pheromone matrix
    pheromone_matrix = np.ones((num_cities, num_cities))

    # Run ACO algorithm for 100 iterations
    for i in range(100):
        # Send ants to find routes
        routes = []
        for _ in range(num_ants):
            route = np.random.randint(num_cities)
            unvisited = set(range(num_cities))
            unvisited.remove(route)
            while unvisited:
                # Select next city using pheromone and heuristic information
                probabilities = pheromone_matrix[route] ** alpha * (1 / _distances[route, :])[unvisited] ** beta
                next_city = np.random.choice(list(unvisited), p=probabilities / np.sum(probabilities))
                route.append(next_city)
                unvisited.remove(next_city)
            routes.append(route)

        # Update pheromone matrix
        for route in routes:
            for i in range(num_cities):
                pheromone_matrix[route[i], route[(i + 1) % num_cities]] += 1

    # Return the best route found by ACO
    best_route = routes[np.argmin([calculate_route_distance(r, _distances) for r in routes])]
    return best_route

