import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Define a heuristic function to guide the search
def heuristic(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Estimates the total distance of a route.

    Args:
        route: A permutation of cities.
        distances: A square matrix of distances between cities.

    Returns:
        The estimated total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
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
