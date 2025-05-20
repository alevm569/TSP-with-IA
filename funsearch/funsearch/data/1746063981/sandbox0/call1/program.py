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

    # Implement a new heuristic called "greedy-insertion"
    def greedy_insertion(distances: np.ndarray) -> list[int]:
        # Initialize an empty route
        route = []
        # Mark all cities as unvisited
        unvisited = list(range(len(distances)))

        # Start from the first city
        current_city = 0
        unvisited.remove(current_city)

        # Iterate until all cities are visited
        while unvisited:
            # Find the city with the shortest distance from the current city
            min_distance = float('inf')
            next_city = None
            for city in unvisited:
                if distances[current_city][city] < min_distance:
                    min_distance = distances[current_city][city]
                    next_city = city

            # Add the next city to the route
            route.append(next_city)
            # Remove the next city from the list of unvisited cities
            unvisited.remove(next_city)

            # Update the current city
            current_city = next_city

        # Return the route as a tuple
        return tuple(route)

    # Use greedy-insertion as the heuristic
    route = greedy_insertion(_distances)

    # Return the route

