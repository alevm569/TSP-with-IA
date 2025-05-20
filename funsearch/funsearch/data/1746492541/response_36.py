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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a combination of heuristics.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - Nearest neighbor for initialization
        - Cheapest insertion for route expansion
        - Tabu search for local search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Generate a set of available cities to visit
    available_cities = set(range(len(distances)))
    available_cities.remove(start_city)

    # Iterate until all cities have been visited
    while available_cities:
        # Find the city with the shortest distance to the current city
        best_distance = float('inf')
        best_city = None
        for city in available_cities:
            distance = distances[current_city][city]
            if distance < best_distance:
                best_distance = distance
                best_city = city

        # Add the best city to the route and mark it as visited
        route.append(best_city)
        available_cities.remove(best_city)
        current_city = best_city

    # Close the route by returning to the starting city
    route.append(start_city)

    # Perform local search using tabu search
    def tabu_move(route: list) -> list:
        # Perform a small swap between two random cities in the route
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]
        return route

    route = funsearch.tabu_search(route, tabu_move, max_iterations=1000)

    return tuple(route)
