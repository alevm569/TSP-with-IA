def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can explore a large number of possible solutions and find the best one.

    # Example using simulated annealing:
    from funsearch import simulated_annealing

    def fitness(route: ndarray) -> float:
        return calculate_route_distance(route, _distances)

    best_route = simulated_annealing(fitness, len(_distances), max_iterations=10000)

    # Return the best route as a tuple of city indices.
    return tuple(best_route)
