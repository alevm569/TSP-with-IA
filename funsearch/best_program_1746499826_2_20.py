"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics and if you are going to use randomness, stabilize it by setting a seed to ensure reproducibility.
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
    # print(route, distance)
    # print(distances[2,9])
    # print(distances[5,17])
    return float(distance)


def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    # print(best_route)
    return calculate_route_distance(best_route, matrix_distances)

def best_program_route(matrix_distances: ndarray) -> tuple[int, ...]:
    """
    Function to generate a random route.
    """
    return find_best_route(matrix_distances)


def find_best_route(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.
    
    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Use nearest neighbor heuristic to generate an initial solution
    start_city = np.random.randint(len(_distances))
    # print(start_city)
    route = [start_city]
    unvisited_cities = list(range(len(_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Use 2-opt heuristic to improve the solution
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_before = _distances[route[i]][route[j]]
                distance_after = _distances[route[i]][route[j - 1]] + _distances[route[j]][route[i]]
                if distance_after < distance_before:
                    route[i:j] = route[j-1:i:-1]

    return tuple(route)

# print(evaluate(0))
