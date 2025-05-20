def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt.
    """

    # Perform nearest neighbor to generate an initial solution
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        closest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(closest_city)
        remaining_cities.remove(closest_city)
        current_city = closest_city

    # Perform 2-opt to improve the solution
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = _distances[route[i]][route[j]] + _distances[route[(j - 1) % len(route)]][route[(j + 1) % len(route)]]
            distance_reversed = _distances[route[i]][route[(j - 1) % len(route)]] + _distances[route[j]][route[(j + 1) % len(route)]]
            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
