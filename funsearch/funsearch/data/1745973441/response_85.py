def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic, e.g., using a genetic algorithm or ant colony optimization.
    # Consider incorporating multiple heuristics for diversification.

    # Example using a genetic algorithm:
    from pymoo.algorithms.moo.age import AGEMOEA

    # Create a genetic algorithm object
    algorithm = AGEMOEA()

    # Run the algorithm to find the best route
    res = algorithm.solve(funsearch.OptimizeRoute(_distances))

    # Return the best route found
    return res.X.astype(int)
