def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a metaheuristic optimization algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can help find optimal or near-optimal solutions to the TSP problem.

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Use a simulated annealing algorithm to find the best route.
    best_route = funsearch.simulated_annealing(_distances, starting_route=list(range(len(_distances))))

    # Return the best route as a tuple of city indices.
    return tuple(best_route)
