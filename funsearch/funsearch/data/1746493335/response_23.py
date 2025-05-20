import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize variables
    num_cities = len(_distances)
    best_route = None
    best_distance = float('inf')

    # Generate random starting point
    start_city = np.random.randint(num_cities)

    # Create a set of unvisited cities
    unvisited_cities = set(range(num_cities))
    unvisited_cities.remove(start_city)

    # Start the route from the starting city
    current_city = start_city
    route = [current_city]

    # Iterate until all cities are visited
    while unvisited_cities:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(start_city)

    # Calculate the total distance of the route
    distance = calculate_route_distance(route, _distances)

    # Update the best route if necessary
    if distance < best_distance:
        best_route = route
        best_distance = distance

    return best_route

def calculate_route_distance(route: list[int], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
