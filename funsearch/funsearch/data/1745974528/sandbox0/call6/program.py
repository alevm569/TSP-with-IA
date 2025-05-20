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

    # Implement a hybrid heuristic that combines different strategies:
    # - Use nearest neighbor to initialize a partial route.
    # - Apply a 2-opt heuristic iteratively to refine the route.

    # Initialize a partial route using the nearest neighbor strategy.
    start_city = 0
    current_city = start_city
    partial_route = [current_city]

    while len(partial_route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in partial_route:
            partial_route.append(nearest_city)
            current_city = nearest_city

    # Refine the route using the 2-opt heuristic.
    for _ in range(10):  # Iterate 10 times for improvement
        for i in range(len(partial_route)):
            for j in range(i + 1, len(partial_route)):
                # Calculate the distance of the current route.
                total_distance = sum(_distances[partial_route[i]][partial_route[(i + 1) % len(partial_route)]])

                # Try reversing the segment between cities i and j.
                reversed_segment = partial_route[i:j+1][::-1]
                total_distance_reversed = sum(_distances[partial_route[i]][reversed_segment[0]])
                for k in range(1, len(reversed_segment)):
                    total_distance_reversed += _distances[reversed_segment[k-1]][reversed_segment[k]]
                total_distance_reversed += sum(_distances[reversed_segment[-1]][partial_route[(j+1) % len(partial_route)]])

                # If reversing the segment improves the distance, update the route.
                if total_distance_reversed < total_distance:
                    partial_route[i:j+1] = reversed_segment

    # Add the starting city to the end of the route.
    partial_route.append(start_city)

    return tuple(partial_route)

