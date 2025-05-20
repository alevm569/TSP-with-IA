def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with local search."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform local search
    for _ in range(100):
        # Randomly swap two cities in the route
        i, j = np.random.randint(0, len(_distances), 2)
        route[i], route[j] = route[j], route[i]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            # If not, reverse the swap
            route[i], route[j] = route[j], route[i]

    return route
