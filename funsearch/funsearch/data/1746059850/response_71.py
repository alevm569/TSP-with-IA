def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combine existing ones to improve performance.
    # Consider using metaheuristics such as simulated annealing or genetic algorithms.

    # Example using simulated annealing:
    from funsearch.algorithms.sa import SimulatedAnnealing

    # Create a simulated annealing object with appropriate parameters.
    sa = SimulatedAnnealing(func=evaluate, state=find_best_route_v2(_distances))

    # Run the algorithm until convergence or maximum iterations reached.
    best_route = sa.run()

    return best_route
