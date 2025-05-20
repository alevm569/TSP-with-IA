def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid approach that combines local search with a simulated annealing heuristic.
    # Initialize a random route.
    route = np.random.permutation(len(_distances))

    # Run simulated annealing.
    best_distance = calculate_route_distance(route, _distances)
    for temperature in np.linspace(100, 0.1, 100):
        # Generate a new route by swapping two cities.
        i, j = np.random.randint(len(_distances), size=2)
        new_route = route.copy()
        new_route[i], new_route[j] = new_route[j], new_route[i]

        # Calculate the distance of the new route.
        new_distance = calculate_route_distance(new_route, _distances)

        # Accept the new route if it is better or with a probability based on the temperature.
        if new_distance < best_distance or np.random.rand() < np.exp(-(new_distance - best_distance) / temperature):
            route = new_route
            best_distance = new_distance

    # Perform local search on the best route found by simulated annealing.
    for _ in range(100):
        i, j = np.random.randint(len(_distances), size=2)
        new_route = route.copy()
        new_route[i], new_route[j] = new_route[j], new_route[i]

        # Calculate the distance of the new route.
        new_distance = calculate_route_distance(new_route, _distances)

        # Accept the new route if it is better.
        if new_distance < best_distance:
            route = new_route
            best_distance = new_distance

    # Return the best route as a tuple of city indices.
    return route
