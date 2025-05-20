def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to find the best route.
    # Genetic algorithms are a powerful tool for solving optimization problems.
    # They can be used to find the best route by iteratively evolving a population of routes.
    # Each route is evaluated based on its total distance.
    # The best route is then selected to be the parent of the next generation of routes.
    # This process is repeated until the best route is found.

    # Create a genetic algorithm object.
    ga = funsearch.GeneticAlgorithm(
        evaluate,
        funsearch.TSPGeneticAlgorithmProblem(_distances),
        population_size=100,
        generations=100,
        tournament_size=3,
        mutation_probability=0.1,
        crossover_probability=0.8,
    )

    # Run the genetic algorithm.
    ga.run()

    # Return the best route found by the genetic algorithm.
    return ga.best_solution
