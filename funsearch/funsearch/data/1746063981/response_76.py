def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a combination of genetic algorithms and local search to find a near-optimal solution.
    """

    # Create a genetic algorithm solver
    solver = funsearch.GAOptimizer(len(_distances))

    # Define the fitness function
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = solver.solve(fitness)

    # Perform local search to improve the solution
    best_route = local_search(best_route, _distances)

    return best_route
