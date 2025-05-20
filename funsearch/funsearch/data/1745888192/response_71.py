def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using genetic algorithm.

    Uses a genetic algorithm to find a permutation of cities that minimizes the total route distance.
    """

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Define the genetic algorithm parameters
    population_size = 50
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create the genetic algorithm object
    ga = funsearch.GA(fitness, population_size, generations, crossover_rate, mutation_rate)

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route found
    return best_route
