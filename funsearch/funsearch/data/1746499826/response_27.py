import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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

    Strategy:
    - Use a hybrid heuristic combining the nearest neighbor and cheapest insertion algorithms.
    - Implement a local search optimization using the 2-opt move.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random permutation of cities
    cities = np.random.permutation(np.arange(len(matrix_distances)))

    # Nearest neighbor heuristic
    current_city = cities[0]
    route = [current_city]
    remaining_cities = set(cities[1:])

    while remaining_cities:
        # Find the closest city not already in the route
        next_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Cheapest insertion heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Check if inserting city i between j-1 and j improves the route distance
            if calculate_route_distance(route[:j] + route[i:i+1] + route[j:], matrix_distances) < calculate_route_distance(route, matrix_distances):
                route = route[:j] + route[i:i+1] + route[j:]

    # Local search optimization using 2-opt move
    best_distance = calculate_route_distance(route, matrix_distances)
    best_route = route.copy()

    while True:
        # Generate a random pair of cities in the route
        i, j = np.random.randint(0, len(route), 2)

        # Apply the 2-opt move
        route[i], route[j] = route[j], route[i]

        # Check if the new route distance is better
        new_distance = calculate_route_distance(route, matrix_distances)
        if new_distance < best_distance:
            best_distance = new_distance
            best_route = route.copy()
        else:
            # Reverse the move if it doesn't improve the route distance
            route[i], route[j] = route[j], route[i]

        # Stop if no further improvements can be made
        if new_distance == best_distance:
            break

    return best_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
