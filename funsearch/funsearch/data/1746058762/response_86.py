def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a metaheuristic search algorithm, such as simulated annealing or ant colony optimization.
    # Initialize a population of candidate routes.
    # Evaluate the fitness of each route using the total distance.
    # Repeat until a satisfactory solution is found.

    # Example using simulated annealing:
    from scipy.optimize import minimize

    def fitness(route):
        return calculate_route_distance(route, _distances)

    initial_route = np.random.permutation(np.arange(len(_distances)))
    result = minimize(fitness, initial_route, method='sa')
    return result.x.astype(int)
