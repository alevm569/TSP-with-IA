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
    """Improved version of `find_best_route_v2` using a heuristic search."""

    # Use a constraint programming solver
    solver = funsearch.Solver()

    # Create variables for the route
    cities = list(range(len(_distances)))
    route = [solver.IntVar(city, city) for city in cities]

    # Add constraints to ensure the route is valid
    solver.Add(funsearch.AllDifferent(route))
    solver.Add(route[0] == 0)  # Start from city 0
    solver.Add(route[-1] == 0)  # End at city 0

    # Set the objective function to minimize the total distance
    solver.Minimize(funsearch.Sum([
        _distances[route[i]][route[(i + 1) % len(route)]]
        for i in range(len(route))
    ]))

    # Use the heuristic function to guide the search
    solver.Search(funsearch.HeuristicSearch(heuristic))

    # Solve the problem
    solver.Solve()

    # Return the optimal route
    return tuple(solver.Value(city) for city in route)

