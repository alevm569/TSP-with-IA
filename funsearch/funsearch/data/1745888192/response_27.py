def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize a random permutation of cities
    current_route = np.random.permutation(len(_distances))

    # Perform a local search to find a better route
    for _ in range(100):
        # Randomly swap two cities in the route
        i, j = np.random.randint(0, len(_distances), size=2)
        current_route[i], current_route[j] = current_route[j], current_route[i]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(current_route, _distances)

        # If the new route is better, keep it
        if new_distance < calculate_route_distance(current_route, _distances):
            current_route = new_route

    # Return the best route found
    return current_route
