def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a hybrid approach combining two heuristics:
    # 1. Nearest neighbor heuristic to generate an initial route.
    # 2. 2-opt heuristic to improve the route by swapping two consecutive city pairs.

    # Generate an initial route using the nearest neighbor heuristic.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Improve the route using the 2-opt heuristic.
    improved = True
    while improved:
        improved = False
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                distance_difference = _distances[route[i]][route[j]] - (_distances[route[(i - 1) % len(route)]][route[i]] + _distances[route[j]][route[(j + 1) % len(route)]])
                if distance_difference < 0:
                    route[i], route[j] = route[j], route[i]
                    improved = True

    return tuple(route)
