import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Find a permutation of cities that minimizes the total route distance.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Simulated annealing (SA) with probability of accepting worse solutions
    - Local search to explore neighboring solutions
    - 2-opt neighborhood operator

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize random route
    num_cities = len(distances)
    initial_route = np.random.permutation(num_cities)

    # Create an SA object
    sa = funsearch.SimulatedAnnealing(initial_route)

    # Define the neighborhood operator
    def two_opt(route):
        # Randomly select two cities and reverse the subsequence between them
        i, j = np.random.randint(num_cities, size=2)
        route[i:j+1] = route[i:j+1][::-1]
        return route

    # Set the neighborhood operator and run the SA algorithm
    sa.set_neighborhood_operator(two_opt)
    best_route = sa.run()

    # Perform local search to refine the solution
    local_search = funsearch.LocalSearch(best_route)
    local_search.set_distance_function(calculate_route_distance(distances))
    local_search.run()

    return local_search.best_solution

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
