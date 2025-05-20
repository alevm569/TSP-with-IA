def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform a nearest neighbor heuristic to find an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform a 2-opt heuristic to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j + 1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
