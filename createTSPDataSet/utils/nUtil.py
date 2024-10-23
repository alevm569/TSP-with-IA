from typing import List, Tuple
import numpy as np

from createTSPDataSet.utils.distanceUtil import calculate_path_distance


# find the best path using the nearest neighbor heuristic

# city: dict[str, tuple[float, float]], ex: {'A': (0.0, 0.0), 'B': (1.0, 1.0), 'C': (2.0, 2.0)}
# distances: dict[tuple[str, str], float], ex: {('A', 'B'): 1.4142135623730951, ('A', 'C'): 2.8284271247461903, ('B', 'C'): 1.4142135623730951}

def find_nearest_neighbor_path_solution(ciudad, distances, seed: int = 123) -> List[str]:
    np.random.seed(seed)
    cities = list(ciudad.keys())
    # chose the first city randomly
    first_city = np.random.choice(cities)
    cities.remove(first_city)
    path = [first_city]
    current_city = first_city
    while cities:
        next_city = min(cities, key=lambda city: distances[(current_city, city)])
        cities.remove(next_city)
        path.append(next_city)
        current_city = next_city
    # add first city to close the path
    path.append(first_city)
    return path


# find a best route by using 2-opt algorithm for a given path
# city: dict[str, tuple[float, float]], ex: {'A': (0.0, 0.0), 'B': (1.0, 1.0), 'C': (2.0, 2.0)}
# distances: dict[tuple[str, str], float], ex: {('A', 'B'): 1.4142135623730951, ('A', 'C'): 2.8284271247461903, ('B', 'C'): 1.4142135623730951}

def find_best_route_2opt(distances, path: List[str], seed: int = 123) -> Tuple[List[str], float]:
    np.random.seed(seed)
    best_path = path
    best_distance = calculate_path_distance(distances, path)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(path) - 2):
            for j in range(i + 1, len(path)):
                if j - i == 1:
                    continue
                new_path = path.copy()
                new_path[i:j] = path[j - 1:i - 1:-1]
                new_distance = calculate_path_distance(distances, new_path)
                if new_distance < best_distance:
                    best_path = new_path
                    best_distance = new_distance
                    improved = True
                    path = best_path
    return best_path, best_distance