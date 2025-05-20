def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or hybrid of heuristics here.
    # Consider using a metaheuristic optimization algorithm such as Simulated Annealing or Particle Swarm Optimization.

    # Example using Simulated Annealing:
    from funsearch.metaheuristic import SimulatedAnnealing

    def route_fitness(route):
        return calculate_route_distance(route, _distances)

    initial_route = np.random.permutation(len(_distances))
    sa = SimulatedAnnealing(route_fitness, initial_route)
    best_route = sa.optimize()

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
