def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with simulated annealing."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Initialize the temperature
    temperature = 100

    # Perform simulated annealing
    while temperature > 0.1:
        # Generate a new route by swapping two cities
        i, j = np.random.randint(0, len(_distances), 2)
        new_route = route.copy()
        new_route[i], new_route[j] = new_route[j], new_route[i]

        # Calculate the difference in distance between the two routes
        distance_difference = calculate_route_distance(new_route, _distances) - calculate_route_distance(route, _distances)

        # Accept the new route with a probability based on the temperature
        if distance_difference < 0 or np.random.rand() < np.exp(-distance_difference / temperature):
            route = new_route

        # Decrease the temperature
        temperature *= 0.99

    return route
