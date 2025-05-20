def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic here, for example:
    # - Ant colony optimization
    # - Genetic algorithm
    # - Heuristic search with local search

    # Example using ant colony optimization:
    from antpy import ACO
    aco = ACO(distance_matrix=_distances)
    best_route = aco.run()

    return best_route
