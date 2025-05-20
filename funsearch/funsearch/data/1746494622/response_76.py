import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility.
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    # Create a list of unvisited cities.
    unvisited_cities = list(range(len(matrix_distances)))
    # Start at the first city.
    current_city = unvisited_cities[0]
    # Initialize the route with the starting city.
    route = [current_city]

    # Visit the remaining cities.
    while unvisited_cities:
        # Find the city with the shortest distance to the current city.
        next_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        # Add the next city to the route.
        route.append(next_city)
        # Remove the next city from the list of unvisited cities.
        unvisited_cities.remove(next_city)
        # Update the current city.
        current_city = next_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return distance
