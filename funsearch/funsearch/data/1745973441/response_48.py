def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Initialize population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define fitness function
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=1000)

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3` using hybrid approach."""

    # Combine genetic algorithm with local search
    best_route = find_best_route_v3(_distances)
    best_distance = fitness(best_route)

    while True:
        # Perform local search on best route
        improved_route = local_search(best_route, _distances)
        improved_distance = fitness(improved_route)

        if improved_distance < best_distance:
            best_route = improved_route
            best_distance = improved_distance
        else:
            break

    return best_route
