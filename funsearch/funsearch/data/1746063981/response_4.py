def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Further improved version of `find_best_route_v1`."""

    # Perform a local search optimization using the 2-opt heuristic
    best_route = find_best_route_v1(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        improved = False
        for i in range(len(best_route)):
            for j in range(i + 2, len(best_route)):
                new_route = best_route[:i] + best_route[j:i:-1] + best_route[j + 1:]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route
                    improved = True

        if not improved:
            break

    return best_route
