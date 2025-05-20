def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use nearest neighbor heuristic to generate an initial solution
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Add the starting city back to the route
    route.append(route[0])

    # Use 2-opt local search to improve the solution
    for _ in range(100):
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    return tuple(route)
