def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic that combines multiple approaches:
    # - Nearest neighbor to generate an initial solution
    # - Cheapest insertion to refine the solution
    # - Local search to optimize the solution further

    # Example hybrid heuristic using nearest neighbor and cheapest insertion:
    n = len(_distances)
    route = list(range(n))
    np.random.shuffle(route)

    for i in range(n):
        current_city = route[i]
        nearest_city = np.argmin(_distances[current_city])
        route.insert(i + 1, nearest_city)

    for i in range(n):
        min_distance = np.inf
        min_city = None

        for j in range(n):
            if j not in route:
                distance = _distances[route[i]][j]
                if distance < min_distance:
                    min_distance = distance
                    min_city = j

        route.insert(i + 1, min_city)

    return tuple(route)
