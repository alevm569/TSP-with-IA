def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic, e.g., simulated annealing or genetic algorithm.
    # Use the `funsearch` library to implement the search algorithm.

    # Create a new search problem using the distance matrix.
    problem = funsearch.SearchProblem(
        domain=funsearch.DiscreteDomain(len(_distances)),
        initial_state=tuple(range(len(_distances))),
        goal_test=lambda s: len(s) == len(_distances) + 1,
        transition_function=funsearch.random_permutation_transition,
        cost_function=calculate_route_distance(_distances),
    )

    # Run the search algorithm.
    best_route = funsearch.simulated_annealing(problem)

    return best_route
