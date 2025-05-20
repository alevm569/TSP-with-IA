def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

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

    # Perform 2-opt local search
    for i in range(len(_distances)):
        for j in range(i + 1, len(_distances)):
            # Create a new route by reversing the subroute between i and j
            new_route = np.copy(route)
            new_route[i:j+1] = route[j:i:-1]

            # Check if the new route is better
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
