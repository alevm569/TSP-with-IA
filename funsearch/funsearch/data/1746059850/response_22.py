def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a hybrid approach using nearest neighbor and 2-opt heuristics
    # Initialize the route with the nearest neighbor strategy
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply the 2-opt heuristic to improve the route
    def two_opt(route):
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    route = new_route
        return route

    route = two_opt(route)

    return route
