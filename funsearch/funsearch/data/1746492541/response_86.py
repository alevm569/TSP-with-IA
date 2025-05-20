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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function employs a hybrid heuristic that combines the following steps:

    1. **Nearest Neighbor:** Start from the first city and iteratively select the closest unvisited city.
    2. **Greedy Insertion:** Randomly select a city to insert at each step, ensuring it improves the current route distance.
    3. **Tabu Search:** Employ a tabu search algorithm to explore nearby routes, avoiding repeated or recently visited cities.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize route and visited set
    n = len(distances)
    route = np.zeros(n, dtype=int)
    visited = set()

    # Start from the first city
    current_city = 0
    visited.add(current_city)

    # Nearest Neighbor to generate initial route
    for i in range(n):
        closest_city = np.argmin(distances[current_city])
        while closest_city in visited:
            closest_city = np.argmin(distances[current_city][np.isin(np.arange(n), visited, invert=True)])
        route[i] = closest_city
        visited.add(closest_city)
        current_city = closest_city

    # Greedy insertion with tabu search
    for i in range(n, 2*n):
        best_distance = np.inf
        best_city = None

        for city in range(n):
            if city in visited:
                continue

            temp_route = route.copy()
            temp_route[i % n] = city
            distance = calculate_route_distance(temp_route, distances)

            if distance < best_distance:
                best_distance = distance
                best_city = city

        # Tabu search to explore nearby routes
        best_route = temp_route
        tabu_list = set()

        for _ in range(100):  # Adjust the number of tabu iterations as needed
            neighbor_routes = generate_neighbor_routes(best_route)
            best_neighbor = min(neighbor_routes, key=calculate_route_distance, default=best_route)

            if best_neighbor not in tabu_list and calculate_route_distance(best_neighbor, distances) < best_distance:
                best_distance = calculate_route_distance(best_neighbor, distances)
                best_route = best_neighbor
                tabu_list.add(best_route)

        route[i % n] = best_city

    # Return to the starting city
    route = np.append(route, route[0])

    return route


# Helper functions

def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance

def generate_neighbor_routes(route: np.ndarray) -> list[np.ndarray]:
    """Generates a list of neighboring routes by swapping two cities."""
    neighbors = []
    for i in range(len(route)):
        for j in range(i+1, len(route)):
            temp_route = route.copy()
            temp_route[i], temp_route[j] = temp_route[j], temp_route[i]
            neighbors.append(temp_route)
    return neighbors
