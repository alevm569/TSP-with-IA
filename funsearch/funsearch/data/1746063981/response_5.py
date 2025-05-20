def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid approach."""

    # Use nearest neighbor to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Perform local search to refine the route
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    return route
