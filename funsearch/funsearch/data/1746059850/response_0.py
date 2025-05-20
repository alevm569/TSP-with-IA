def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic here, such as:
    # - Ant Colony Optimization (ACO)
    # - Genetic Algorithm (GA)
    # - Tabu Search

    # Example using ACO:
    num_cities = len(_distances)
    ACO = funsearch.algorithms.ant_colony.AntColony(num_cities, _distances)
    best_route = ACO.run()

    return best_route
