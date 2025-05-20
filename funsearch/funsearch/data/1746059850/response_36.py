def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a genetic algorithm object.
    ga = funsearch.GA(population_size, generations, mutation_rate, crossover_rate)

    # Create a fitness function.
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm to find the best route.
    best_route = ga.run(fitness, len(_distances))

    # Return the best route.
    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
