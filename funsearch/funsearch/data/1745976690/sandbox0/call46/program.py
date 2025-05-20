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

    # Initialize the search space
    n = len(_distances)
    search_space = list(range(n))

    # Create a fitness function to minimize the total distance
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Use the genetic algorithm with a hybrid heuristic
    algorithm = funsearch.genetic.GeneticAlgorithm(
        population_size=100,
        generations=100,
        mutation_rate=0.1,
        crossover_rate=0.8,
        fitness_function=fitness,
        heuristic=funsearch.genetic.hybrid_heuristic([
            funsearch.genetic.nearest_neighbor_heuristic,
            funsearch.genetic.cheapest_insertion_heuristic,
            funsearch.genetic.local_search_heuristic,
        ]),
        search_space=search_space,
    )

    # Run the algorithm and find the best route
    best_route = algorithm.run()[0]

