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
    """Improved version of `find_best_route_v1` with genetic algorithm."""

    # Create a population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Evaluate the fitness of each route
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Perform genetic algorithm
    for generation in range(100):
        # Selection
        parents = np.random.choice(population, size=50, replace=False, p=fitness / np.sum(fitness))

        # Crossover
        children = []
        for i in range(0, len(parents), 2):
            parent1 = parents[i]
            parent2 = parents[i + 1]
            crossover_point = np.random.randint(1, len(_distances))
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutation
            mutation_rate = 0.01
            for i in range(len(child)):
                if np.random.rand() < mutation_rate:
                    j = np.random.randint(0, len(_distances))
                    child[i], child[j] = child[j], child[i]

            children.append(child)

        # Evaluate the fitness of each child route
        child_fitness = [calculate_route_distance(route, _distances) for route in children]

        # Add the new children to the population
        population.extend(children)
        fitness.extend(child_fitness)

    # Return the route with the lowest fitness
    best_route_index = np.argmin(fitness)
    return population[best_route_index]

