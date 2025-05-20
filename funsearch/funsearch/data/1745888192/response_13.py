def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    current_city = 0
    route = [current_city]

    while len(route) < num_cities:
        nearest_city = -1
        min_distance = float('inf')

        for i in range(num_cities):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        route.append(nearest_city)
        current_city = nearest_city

    return tuple(route)
