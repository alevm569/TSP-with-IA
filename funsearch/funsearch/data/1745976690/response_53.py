def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the search space
    n = len(_distances)
    search_space = list(range(n))

    # Create a fitness function to minimize the total distance
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Use the genetic algorithm with a hybrid heuristic
    algorithm = funsearch.genetic.GeneticAlgorithm(
        population_size=100,
        generations=100,
        mutation_rate=0.1,
        crossover_rate=0.8,
        fitness_function=fitness,
        heuristic=funsearch.genetic.hybrid_heuristic([
            funsearch.genetic.nearest_neighbor_heuristic,
            funsearch.genetic.cheapest_insertion_heuristic,
            funsearch.genetic.local_search_heuristic,
        ]),
        search_space=search_space,
    )

    # Run the algorithm and find the best route
    best_route = algorithm.run()[0]

    return best_route
