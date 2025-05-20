def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or strategy here.
    # For example, you could use a hybrid approach that combines different heuristics.

    # Apply the 2-opt heuristic to the current route.
    current_route = find_best_route_v1(_distances)
    best_distance = calculate_route_distance(current_route, _distances)

    while True:
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                new_route = current_route[:i] + current_route[j:i:-1] + current_route[j + 1:]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    current_route = new_route

        # Check if the route has improved.
        if best_distance == calculate_route_distance(current_route, _distances):
            break

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    return current_route
