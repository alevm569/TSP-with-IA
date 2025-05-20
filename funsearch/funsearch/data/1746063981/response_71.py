def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or ant colony optimization.
    # Implement a custom fitness function that calculates the total distance of a route.

    # Initialize the metaheuristic algorithm.
    algorithm = funsearch.SimulatedAnnealing()

    # Set the fitness function.
    algorithm.fitness_function = calculate_route_distance

    # Run the metaheuristic algorithm.
    best_route = algorithm.run(len(_distances))

    # Return the best route.
    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""

    total_distance = 0.0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
