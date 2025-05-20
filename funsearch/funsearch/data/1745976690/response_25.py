def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize population of routes using a nearest neighbor strategy
    population = initialize_population()

    # Perform local search to refine the routes
    population = local_search(population, _distances)

    # Return the route with the lowest total distance
    return min(population, key=lambda route: calculate_route_distance(route, _distances))


def initialize_population():
    # ... Code to generate a population of routes using a nearest neighbor strategy
    pass


def local_search(population, _distances):
    # ... Code to perform local search on the population of routes
    pass
