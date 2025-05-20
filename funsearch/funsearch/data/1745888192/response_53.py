def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithms.
    # These algorithms can explore the solution space more effectively and find better solutions.

    # Example using simulated annealing:
    from funsearch.algorithms import simulated_annealing

    def distance_function(route):
        return calculate_route_distance(route, _distances)

    best_route = simulated_annealing(distance_function, len(_distances))

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3`."""

    # Use a hybrid approach that combines different heuristics and metaheuristics.
    # This can further improve the performance of the algorithm.

    # Example using a hybrid approach:
    from funsearch.algorithms import genetic_algorithm

    def distance_function(route):
        return calculate_route_distance(route, _distances)

    best_route = genetic_algorithm(distance_function, len(_distances))

    return best_route
