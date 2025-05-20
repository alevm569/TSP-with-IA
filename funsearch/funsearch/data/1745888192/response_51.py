def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the gene pool
    gene_pool = list(range(len(_distances)))

    # Create the fitness function
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm
    ga = funsearch.GeneticAlgorithm(
        fitness_function=fitness_function,
        gene_pool=gene_pool,
        population_size=100,
        generations=1000,
    )

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route
    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
