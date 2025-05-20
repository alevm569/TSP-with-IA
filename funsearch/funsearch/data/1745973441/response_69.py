def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a combination of local search and genetic algorithm.

    Local search is used to refine the initial solution provided by the genetic algorithm.
    """

    # Use a genetic algorithm to find an initial population of routes.
    population = genetic_algorithm(_distances)

    # Perform local search on each route in the population.
    for route in population:
        route = local_search(route, _distances)

    # Return the best route found.
    return max(population, key=calculate_route_distance)
