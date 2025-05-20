def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.

    # Use the funsearch library to perform metaheuristic optimization.
    problem = funsearch.TSPProblem(distances=_distances)
    algorithm = funsearch.GWOAlgorithm()  # Example metaheuristic algorithm
    solution = funsearch.solve(problem, algorithm)

    # Return the best route as a tuple of city indices.
    return tuple(solution.route)
