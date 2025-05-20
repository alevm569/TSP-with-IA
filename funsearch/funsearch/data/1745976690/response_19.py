def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    # Initialize the route with the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    remaining_cities = list(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply 2-opt local search to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j + 1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
