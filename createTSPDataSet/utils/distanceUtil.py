from typing import List
import datetime as dt

def get_path(edges: dict, initial_city: str, path: List[str]):
    next_node = edges.get(initial_city, "")
    if next_node is None:
        return [next_node]
    elif next_node in path:
        return path
    path.append(next_node)
    return get_path(edges, next_node, path)

def calculate_path_distance(distances: dict, path: List[str]):
    distance = 0
    for i in range(len(path) - 1):
        if path[i] is not None and path[i+1] is not None:
            distance += distances[(path[i], path[i+1])]
    return distance

def delta_time_mm_ss(delta_time: dt.timedelta):
    minutes, seconds = divmod(delta_time.seconds, 60)
    return f"{0 if minutes < 10 else ''}{minutes}:{0 if seconds < 10 else ''}{seconds}"

def get_min_distance(distances: dict):
    min_distance = min(distances.values())
    return min_distance

def get_max_distance(distances: dict):
    max_distance = max(distances.values())
    return max_distance

def get_average_distance(distances: dict):
    avg_distance = sum(distances.values()) / len(distances)
    return avg_distance

def get_mean_distance_for_city(city:str, distances: dict):
    acc_distances = 0
    neighbors = 0
    for k, v in distances.items():
        if city in k:
            acc_distances += v
            neighbors += 1
    return acc_distances / neighbors

def get_average_distance_for_city(distances: dict):
    cities = list(set([city for k in distances.keys() for city in k]))
    best_max_distances = {}
    for city in cities:
        best_max_distances[city] = get_mean_distance_for_city(city, distances)
    return best_max_distances

def get_min_distance_for_city(city:str, distances: dict):
    min_distance = None
    for d in distances.keys():
        if city in d:
            if min_distance is None or distances[d] < min_distance:
                min_distance = distances[d]
    return min_distance

def get_min_distances(distances: dict):
    cities = list(set([city for k in distances.keys() for city in k]))
    min_distances = {}
    for city in cities:
        min_distances[city] = get_min_distance_for_city(city, distances)
    return min_distances

def get_max_distance_for_city(city:str, distances: dict):
    max_distance = None
    for d in distances.keys():
        if city in d:
            if max_distance is None or distances[d] > max_distance:
                max_distance = distances[d]
    return max_distance

def get_max_distances(distances: dict):
    cities = list(set([city for k in distances.keys() for city in k]))
    max_distances = {}
    for city in cities:
        max_distances[city] = get_max_distance_for_city(city, distances)
    return max_distances