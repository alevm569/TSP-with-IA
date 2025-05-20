def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with genetic algorithm."""

    # Define the genetic algorithm parameters
    population_size = 100
    num_generations = 100
    mutation_rate = 0.01

    # Create the genetic algorithm solver
    solver = funsearch.GA(_distances, population_size, num_generations, mutation_rate)

    # Run the solver
    best_route = solver.solve()

    # Return the best route
    return best_route
