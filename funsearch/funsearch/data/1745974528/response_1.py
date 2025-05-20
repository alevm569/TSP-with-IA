def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Further improved version of `find_best_route_v1`."""

    # Apply a hybrid approach combining two heuristics:

    # 1. Greedy nearest neighbor heuristic:
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # 2. 2-opt heuristic:
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = _distances[route[i]][route[j]] + _distances[route[(j - 1) % len(route)]][route[(j + 1) % len(route)]]
            distance_reversed = _distances[route[i]][route[(j - 1) % len(route)]] + _distances[route[j]][route[(j + 1) % len(route)]]

            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i-1:-1]

    return tuple(route)
