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
    """Improved version of `find_best_route_v1` with the following enhancements:

    - Uses a hybrid approach combining the nearest neighbor heuristic with the 2-opt local search algorithm.
    - Implements a constraint satisfaction framework to enforce the requirement that all cities are visited exactly once and return to the starting point.
    """

    # Initialize the constraint satisfaction framework.
    problem = funsearch.Problem()

    # Create variables representing the order in which cities are visited.
    cities = list(range(len(_distances)))
    variables = [problem.add_variable(city) for city in cities]

    # Add constraints to ensure that each city is visited exactly once.
    for city in cities:
        problem.add_constraint(funsearch.AllDifferentConstraint(variables))

    # Add constraints to ensure that the route returns to the starting city.
    problem.add_constraint(funsearch.CircuitConstraint(variables))

    # Define the objective function as the total route distance.
    def total_distance(assignment):
        route = [variable.get_value(assignment) for variable in variables]
        return calculate_route_distance(route, _distances)

    problem.set_objective(total_distance)

    # Use hybrid search with nearest neighbor and 2-opt local search.
    solver = funsearch.HybridSearch(
        funsearch.NearestNeighborHeuristic(), funsearch.TwoOptLocalSearch(), _distances
    )

    # Solve the problem using the constraint satisfaction framework.
    assignment = solver.solve(problem)

    # Return the best route found.

