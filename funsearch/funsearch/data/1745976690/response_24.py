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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt.
    """

    # Initial route using nearest neighbor
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(1, len(_distances)))

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # 2-opt optimization
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            dist_original = _distances[route[i]][route[j]]
            new_route = route[:i] + route[i+j:j:-1] + route[j+1:]
            dist_new = calculate_route_distance(new_route, _distances)
            if dist_new < dist_original:
                route = new_route

    return tuple(route)
