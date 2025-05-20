def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with genetic algorithm."""

    # Initialize population
    population = funsearch.Population(
        size=100,
        genes=funsearch.PermutationGene(len(_distances)),
    )

    # Define fitness function
    fitness_function = funsearch.FitnessMin(_distances, _distances)

    # Run genetic algorithm
    population = funsearch.genetic_algorithm(
        population=population,
        fitness_function=fitness_function,
        max_generations=1000,
        crossover_rate=0.8,
        mutation_rate=0.2,
    )

    # Return best route
    return population.best.genes.values
