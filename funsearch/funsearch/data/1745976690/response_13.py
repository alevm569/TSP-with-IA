def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Apply nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    while len(route) < len(_distances):
        closest_city = np.argmin(_distances[current_city])
        if closest_city not in route:
            route.append(closest_city)
            current_city = closest_city

    # Apply 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = _distances[route[i]][route[j]]
            distance_after = _distances[route[i]][route[j - 1]] + _distances[route[j]][route[i]]
            if distance_after < distance_before:
                route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
