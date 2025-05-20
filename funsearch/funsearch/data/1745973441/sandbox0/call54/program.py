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
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Use a metaheuristic optimization algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can help find optimal or near-optimal solutions to the TSP problem.

    # Use a genetic algorithm to optimize the route.
    num_cities = len(_distances)
    population_size = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    def fitness(route):
        total_distance = 0
        for i in range(num_cities):
            total_distance += _distances[route[i]][route[(i + 1) % num_cities]]
        return total_distance

    # Create an initial population of routes.
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Run the genetic algorithm for a specified number of generations.
    for generation in range(100):
        # Select the fittest routes.
        sorted_routes = sorted(population, key=fitness)
        fittest_routes = sorted_routes[:population_size // 2]

        # Create the next generation of routes.
        new_population = []
        for i in range(population_size):
            # Crossover two parent routes.
            parent1 = np.random.choice(fittest_routes)
            parent2 = np.random.choice(fittest_routes)
            crossover_point = np.random.randint(1, num_cities)
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutate the route.
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(0, num_cities)
                mutation_point2 = np.random.randint(0, num_cities)
                child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            new_population.append(child)

        population = new_population

    # Return the best route.
    best_route = sorted(population, key=fitness)[0]

