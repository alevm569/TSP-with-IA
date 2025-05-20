def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # Consider using a metaheuristic algorithm like simulated annealing or ant colony optimization.

    # Return the best route as a tuple of city indices.
    return tuple(np.random.permutation(len(_distances)))
