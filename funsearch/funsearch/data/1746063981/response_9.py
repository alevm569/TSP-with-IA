def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a metaheuristic algorithm such as simulated annealing or ant colony optimization.
    # These algorithms can explore a wider range of solutions compared to brute force.

    # Example using simulated annealing:
    from funsearch.algorithms.metaheuristics import simulated_annealing

    # Initialize the search algorithm
    sa = simulated_annealing.SimulatedAnnealing(
        initial_state=np.random.permutation(len(_distances)),
        objective_function=calculate_route_distance,
        distance_matrix=_distances
    )

    # Run the algorithm until convergence or maximum iterations reached
    best_route = sa.run()

    return best_route
