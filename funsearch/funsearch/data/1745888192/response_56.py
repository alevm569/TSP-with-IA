def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using simulated annealing."""

    # Initialize the current route
    current_route = np.random.permutation(len(_distances))

    # Set the initial temperature
    temperature = len(_distances) * 10

    # Set the cooling rate
    cooling_rate = 0.99

    # Perform simulated annealing iterations
    while temperature > 1:
        # Generate a new route by swapping two random cities
        index1, index2 = np.random.randint(0, len(_distances), 2)
        new_route = current_route.copy()
        new_route[index1], new_route[index2] = new_route[index2], new_route[index1]

        # Calculate the difference in distance between the two routes
        distance_difference = calculate_route_distance(new_route, _distances) - calculate_route_distance(current_route, _distances)

        # Accept the new route if it improves the distance or with a probability based on the temperature
        if distance_difference < 0 or np.random.rand() < np.exp(-distance_difference / temperature):
            current_route = new_route

        # Cool down the temperature
        temperature *= cooling_rate

    # Return the best route found
    return current_route
