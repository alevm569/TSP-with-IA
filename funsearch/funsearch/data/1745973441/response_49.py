def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can explore a wider range of solutions and potentially find better routes.

    # Example using simulated annealing:
    from funsearch.algorithms import simulated_annealing

    # Define the search space
    start = list(range(len(_distances)))

    # Define the objective function
    def objective(route):
        return calculate_route_distance(route, _distances)

    # Run the algorithm
    best_route = simulated_annealing(objective, start)

    return tuple(best_route)
