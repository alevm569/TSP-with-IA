import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route( _distances: np.ndarray) -> tuple[int, ...]:
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
    # Define the heuristic function
    def heuristic(route: tuple[int, ...]) -> float:
        total_distance = 0
        for i in range(len(route)):
            city1 = route[i]
            city2 = route[(i + 1) % len(route)]
            total_distance += _distances[city1][city2]
        return total_distance

    # Initialize the search algorithm
    search = funsearch.Search(heuristic)

    # Add the initial route
    search.add_route(tuple(range(len(_distances))))

    # Run the search algorithm
    best_route = search.run()

    return best_route


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route`."""
    # Add your improvements here.

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    # Add your further improvements here.
