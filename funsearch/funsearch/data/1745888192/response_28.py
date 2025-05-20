def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or ant colony optimization.
    # These algorithms can explore the solution space more effectively and find better routes.

    # Example using simulated annealing:
    from funsearch.algorithms.metaheuristics import simulated_annealing

    # Define the initial route
    initial_route = np.random.permutation(len(_distances))

    # Run the simulated annealing algorithm
    best_route = simulated_annealing(
        initial_route,
        _distances,
        max_iterations=10000,
        temperature_decay=0.99,
        best_score_callback=print,
    )

    return best_route
