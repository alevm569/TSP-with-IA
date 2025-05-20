def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with 2-opt heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform 2-opt local search
    for _ in range(100):
        # Randomly select two non-adjacent cities
        i, j = np.random.randint(0, len(_distances), 2)
        while i == j or _distances[route[i], route[j]] == 0:
            i, j = np.random.randint(0, len(_distances), 2)

        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            # If not, reverse the swap
            route[i], route[j] = route[j], route[i]

    return route
