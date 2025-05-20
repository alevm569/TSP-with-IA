def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform nearest neighbor heuristic to generate an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Apply 2-opt local search to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            dist1 = _distances[route[i]][route[j]]
            dist2 = _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[(i + 1) % len(route)]]
            if dist2 < dist1:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
