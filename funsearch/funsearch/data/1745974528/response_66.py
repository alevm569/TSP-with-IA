def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with local search."""

    # Generate an initial random route.
    n = len(_distances)
    current_route = np.random.permutation(n)

    # Perform local search.
    for i in range(100):  # Number of iterations
        # Select two random cities in the route.
        a, b = np.random.randint(n, size=2)

        # Swap the two cities in the route.
        current_route[a], current_route[b] = current_route[b], current_route[a]

        # Check if the new route is better.
        if calculate_route_distance(current_route, _distances) < calculate_route_distance(current_route, _distances):
            pass  # Keep the new route.
        else:
            # Restore the original route.
            current_route[a], current_route[b] = current_route[b], current_route[a]

    return current_route
