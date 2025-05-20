def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Create a Genetic Algorithm object
    ga = funsearch.GA(
        population_size=100,
        tournament_size=3,
        crossover_rate=0.8,
        mutation_rate=0.2,
        max_generations=100
    )

    # Define the fitness function
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness, len(_distances))

    # Return the best route
    return best_route
