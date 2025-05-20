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
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a list of candidate routes using the nearest neighbor heuristic.
    candidate_routes = [nearest_neighbor(_distances)]

    # Use a local search algorithm to refine the candidate routes.
    best_route = local_search(candidate_routes[0], _distances)

    return best_route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Constructs a route using the nearest neighbor heuristic."""

    # Start at the first city.
    current_city = 0
    route = [current_city]

    # Visit all other cities.
    remaining_cities = list(range(1, len(_distances)))
    while remaining_cities:
        # Find the nearest city to the current city.
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route.
        route.append(nearest_city)

        # Remove the nearest city from the list of remaining cities.
        remaining_cities.remove(nearest_city)

        # Update the current city.
        current_city = nearest_city

    return tuple(route)


def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """Refines a route using a local search algorithm."""

    # Iterate until no improvements are found.
    while True:
        # Generate a candidate route by swapping two cities.
        i, j = np.random.randint(0, len(route), 2)
        candidate_route = route[:i] + (route[j],) + route[i+1:j] + (route[i],) + route[j+1:]

        # If the candidate route is better, update the best route.
        if calculate_route_distance(candidate_route, _distances) < calculate_route_distance(route, _distances):
            route = candidate_route
        else:
            break

    return route


def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""

    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i+1) % len(route)]]

