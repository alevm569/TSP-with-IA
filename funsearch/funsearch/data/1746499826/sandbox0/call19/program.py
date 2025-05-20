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
    """Improved version of `find_best_route_v1`."""

    # Seed the random number generator for reproducibility
    np.random.seed(42)

    # Implement a hybrid heuristic that combines the nearest neighbor and cheapest insertion algorithms
    def hybrid_heuristic(distances):
        # Start with a random city as the initial city
        current_city = np.random.randint(len(distances))
        route = [current_city]

        # Visit each city once
        remaining_cities = set(range(len(distances))) - {current_city}

        while remaining_cities:
            # Find the nearest unvisited city
            nearest_city = min(remaining_cities, key=lambda c: distances[current_city][c])
            route.append(nearest_city)
            remaining_cities.remove(nearest_city)

            # Find the cheapest city to insert into the route
            cheapest_city = min(route[:-1], key=lambda c: distances[route[-1]][c])
            route.insert(route.index(nearest_city), cheapest_city)

        # Return to the starting city
        route.append(route[0])

        return tuple(route)

    # Use the hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

