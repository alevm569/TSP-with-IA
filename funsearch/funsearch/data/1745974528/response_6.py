def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid approach."""

    # Use the nearest neighbor heuristic to find an initial route.
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply the 2-opt heuristic to improve the route.
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = _distances[route[i]][route[j]]
            distance_reversed = _distances[route[i]][route[(j - 1)]] + _distances[route[j]][route[i]] - distance_original
            if distance_reversed < distance_original:
                route[i:j] = route[i:j][::-1]

    return tuple(route)
