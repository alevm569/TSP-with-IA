import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2, incorporating a hybrid heuristic approach.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function combines nearest neighbor, cheapest insertion, and local search heuristics
    to find a good solution.
    """

    # Initialize a set of unvisited cities
    unvisited_cities = set(range(len(_distances)))

    # Start from the first city
    current_city = 0
    route = [current_city]

    # Loop until all cities are visited
    while unvisited_cities:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

        # Move to the nearest city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    # Perform local search to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap two cities in the route
            route[i], route[j] = route[j], route[i]

            # Calculate the total distance of the new route
            new_distance = calculate_route_distance(route, _distances)

            # If the new route is better, keep it
            if new_distance < calculate_route_distance(route, _distances):
                route = route[:]

    return tuple(route)
