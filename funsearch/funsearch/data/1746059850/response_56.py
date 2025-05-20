def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find a good route
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a genetic algorithm object
    ga = funsearch.GA(population_size=population_size,
                      num_generations=num_generations,
                      crossover_rate=crossover_rate,
                      mutation_rate=mutation_rate)

    # Create a fitness function to evaluate the routes
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness, len(_distances))

    # Return the best route
    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
