def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    unvisited_cities = set(range(num_cities))
    current_city = 0
    route = [current_city]

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        unvisited_cities.remove(nearest_city)
        route.append(nearest_city)
        current_city = nearest_city

    return tuple(route)
