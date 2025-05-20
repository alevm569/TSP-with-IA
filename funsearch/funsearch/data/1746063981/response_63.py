def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can explore the search space more efficiently and find better solutions.

    # Example using simulated annealing:
    from funsearch.algorithms import simulated_annealing

    # Define the initial solution
    initial_route = np.random.permutation(len(_distances))

    # Run the simulated annealing algorithm
    best_route, _ = simulated_annealing(
        initial_route,
        distance_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    return best_route
