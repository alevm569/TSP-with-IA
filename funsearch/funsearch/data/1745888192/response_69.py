def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize a list to store the route
    route = list(range(len(_distances)))

    # Randomly shuffle the list of cities
    np.random.shuffle(route)

    # Use a local search algorithm to improve the route
    for i in range(100):
        # Randomly swap two cities in the route
        a, b = np.random.randint(0, len(_distances), size=2)
        route[a], route[b] = route[b], route[a]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            # Keep the new route
            pass
        else:
            # Restore the old route
            route[a], route[b] = route[b], route[a]

    # Return the best route
    return tuple(route)
