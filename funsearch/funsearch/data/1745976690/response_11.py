def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using metaheuristics."""

    # Define the search space
    search_space = list(range(len(_distances)))

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Use a metaheuristic algorithm, e.g., simulated annealing
    best_route = funsearch.sa.simulated_annealing(fitness, search_space)

    return best_route
