import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the ACO heuristic for route planning
    aco = funsearch.ACO(_distances)
    best_route = aco.find_best_route()

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3`."""

    # Use the genetic algorithm heuristic for route planning
    ga = funsearch.GeneticAlgorithm(_distances)
    best_route = ga.find_best_route()

    return best_route


def find_best_route_v5(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v4`."""

    # Use the tabu search heuristic for route planning
    ts = funsearch.TabuSearch(_distances)
    best_route = ts.find_best_route()

    return best_route
