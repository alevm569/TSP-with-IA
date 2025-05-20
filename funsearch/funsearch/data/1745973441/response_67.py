def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform local search using the 2-opt heuristic
    for _ in range(100):
        # Randomly select two subroutes
        i, j = np.random.randint(0, len(_distances), 2)

        # Apply the 2-opt swap
        route[i:j+1] = route[j:i:-1]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            # If not, reverse the swap
            route[i:j+1] = route[j:i:-1]

    return route
