def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""
    # Perform nearest neighbor to find an initial solution.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Apply 2-opt heuristic to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = _distances[route[i]][route[j]]
            distance_reversed = _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[(i + 1) % len(route)]] - distance_original
            if distance_reversed < distance_original:
                route = route[:i] + route[j:i:-1] + route[j + 1:]

    return tuple(route)
