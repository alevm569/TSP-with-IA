def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use nearest neighbor heuristic to initialize route
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply 2-opt heuristic to improve route quality
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = _distances[route[i]][route[j]]
            route[i+1:j] = reversed(route[i+1:j])
            distance_after = _distances[route[i]][route[j]]
            if distance_after < distance_before:
                break

    # Perform local search to refine route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            temp_route = route[:]
            temp_route[i], temp_route[j] = temp_route[j], temp_route[i]
            if calculate_route_distance(temp_route, _distances) < calculate_route_distance(route, _distances):
                route = temp_route

    return tuple(route)
