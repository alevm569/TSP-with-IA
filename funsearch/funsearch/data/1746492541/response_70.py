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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - **Nearest neighbor:** Start from an initial city and iteratively select the nearest unvisited city.
        - **Tabu search:** Use a tabu list to avoid revisiting recently visited cities.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))
    current_city = start_city
    route = [current_city]

    # Create a tabu list to keep track of recently visited cities
    tabu_list = [start_city]

    # Iterate until all cities are visited
    while len(route) < len(matrix_distances):
        # Get the list of unvisited cities
        unvisited_cities = [city for city in range(len(matrix_distances)) if city not in route]

        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])

        # Check if the nearest city is already in the tabu list
        if nearest_city in tabu_list:
            # If it is, skip it and find the next nearest city
            unvisited_cities.remove(nearest_city)
            nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])

        # Add the nearest city to the route and tabu list
        route.append(nearest_city)
        tabu_list.append(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
