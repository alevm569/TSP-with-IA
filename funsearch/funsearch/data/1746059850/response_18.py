def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a simulated annealing approach."""

    # Create a copy of the distance matrix for ease of use
    distances = np.copy(_distances)

    # Initialize the best route and its distance
    best_route = tuple(range(len(distances)))
    best_distance = calculate_route_distance(best_route, distances)

    # Initialize the current route and distance
    current_route = np.random.permutation(len(distances))
    current_distance = calculate_route_distance(current_route, distances)

    # Set the initial temperature and cooling rate
    temperature = 100
    cooling_rate = 0.99

    # Run the simulated annealing algorithm
    while temperature > 0.001:
        # Generate a new route by swapping two random cities
        i, j = np.random.randint(0, len(distances), 2)
        new_route = current_route.copy()
        new_route[i], new_route[j] = new_route[j], new_route[i]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(new_route, distances)

        # Accept the new route if it's better or with a probability determined by the temperature
        if new_distance < current_distance or np.random.rand() < np.exp(-(new_distance - current_distance) / temperature):
            current_route = new_route
            current_distance = new_distance

        # Update the best route if necessary
        if current_distance < best_distance:
            best_route = current_route
            best_distance = current_distance

        # Cool down the temperature
        temperature *= cooling_rate

    return best_route
