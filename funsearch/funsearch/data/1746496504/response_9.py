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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt.
    """

    # Perform nearest neighbor to get an initial tour
    current_city = 0
    tour = [current_city]
    unvisited_cities = list(range(1, len(_distances)))

    while unvisited_cities:
        next_city = funsearch.nearest_neighbor(current_city, unvisited_cities, _distances)
        tour.append(next_city)
        unvisited_cities.remove(next_city)
        current_city = next_city

    # Perform 2-opt to improve the tour
    funsearch.two_opt(tour, _distances)

    return tuple(tour)
