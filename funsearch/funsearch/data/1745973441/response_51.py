def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with 2-opt heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform 2-opt local search
    for _ in range(100):
        # Randomly select two cities
        i, j = np.random.randint(0, len(_distances), 2)

        # Apply the 2-opt move
        route[i:j+1] = route[j:i-1:-1]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            # If not, reverse the 2-opt move
            route[i:j+1] = route[j:i-1:-1]

    return route
