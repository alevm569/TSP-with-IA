def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm, to find the optimal route.
    # These algorithms can explore different solutions and find the best one.

    # Example using simulated annealing:
    from funsearch.algorithms.metaheuristic import SimulatedAnnealing

    # Initialize the algorithm
    sa = SimulatedAnnealing(temperature=100, cooling_rate=0.9)

    # Run the algorithm to find the best route
    best_route = sa.optimize(distance_function=calculate_route_distance, distance_matrix=_distances)

    return best_route
