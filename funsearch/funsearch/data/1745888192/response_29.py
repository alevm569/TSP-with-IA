def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform local search using a 2-opt heuristic
    route = find_best_route_v2(_distances)
    best_distance = calculate_route_distance(route, _distances)

    while True:
        improved = False

        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance
                    improved = True

        if not improved:
            break

    return route
