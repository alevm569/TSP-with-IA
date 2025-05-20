def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    # Initialize a starting route using nearest neighbor
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Perform 2-opt local search to optimize the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i:j+1] = new_route[j:i:-1]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
