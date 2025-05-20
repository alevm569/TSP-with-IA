def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform local search using the 2-opt heuristic
    current_route = find_best_route_v1(_distances)
    best_distance = calculate_route_distance(current_route, _distances)

    while True:
        improved = False

        for i in range(len(current_route)):
            for j in range(i + 2, len(current_route)):
                new_route = current_route[:i] + current_route[i+1:j] + current_route[i:i+1] + current_route[j:]

                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    current_route = new_route
                    improved = True

        if not improved:
            break

    return current_route
