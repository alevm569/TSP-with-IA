def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Further improved version of `find_best_route_v1`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm
    # to search for the optimal route.

    # Example using simulated annealing:
    from funsearch import SimulatedAnnealing

    def route_distance(route):
        return calculate_route_distance(route, _distances)

    sa = SimulatedAnnealing(route_distance)
    best_route = sa.solve(initial_state=np.random.permutation(len(_distances)))

    return best_route


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Yet another improved version of `find_best_route_v2`."""

    # Use a hybrid approach that combines different heuristics and metaheuristics.

    # Example using a hybrid approach:
    from funsearch import GeneticAlgorithm

    def route_distance(route):
        return calculate_route_distance(route, _distances)

    ga = GeneticAlgorithm(route_distance)
    best_route = ga.solve(initial_population=np.random.permutation(len(_distances), size=(100, len(_distances))))

    return best_route
