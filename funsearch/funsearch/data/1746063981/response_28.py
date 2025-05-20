def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can explore a broader range of solutions than brute force methods.

    # Define a search space where each city is represented by an integer.
    search_space = list(range(len(_distances)))

    # Use a simulated annealing algorithm to find the best route.
    best_route = funsearch.simulated_annealing(
        search_space=search_space,
        objective_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3`."""

    # Use a genetic algorithm to find the best route.
    # Genetic algorithms can be particularly effective for optimization problems.

    # Define a search space where each city is represented by an integer.
    search_space = list(range(len(_distances)))

    # Use a genetic algorithm to find the best route.
    best_route = funsearch.genetic_algorithm(
        search_space=search_space,
        objective_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    return best_route
