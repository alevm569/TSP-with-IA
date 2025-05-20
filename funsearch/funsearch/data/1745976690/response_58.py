def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using simulated annealing."""

    # Initialize random route
    route = np.random.permutation(len(_distances))

    # Set initial temperature and cooling rate
    temperature = 100
    cooling_rate = 0.99

    # Run simulated annealing algorithm
    while temperature > 0.1:
        # Generate a new route by swapping two cities
        i, j = np.random.randint(len(_distances), size=2)
        new_route = route.copy()
        new_route[i], new_route[j] = new_route[j], new_route[i]

        # Calculate the difference in route distance
        distance_difference = calculate_route_distance(new_route, _distances) - calculate_route_distance(route, _distances)

        # Accept the new route if it improves the distance or with a probability based on the temperature
        if distance_difference < 0 or np.random.rand() < np.exp(-distance_difference / temperature):
            route = new_route

        # Cool down the temperature
        temperature *= cooling_rate

    return route
