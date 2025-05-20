def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Create a genetic algorithm search object
    search = funsearch.GeneticSearch(
        funsearch.TSP(distances=_distances),
        pop_size=100,
        max_generations=1000,
        tournament_size=3,
        crossover_rate=0.8,
        mutation_rate=0.2,
        elitism=True,
    )

    # Run the search
    best_route = search.run()[0]

    return best_route
