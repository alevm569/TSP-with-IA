def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the genetic algorithm to the TSP problem
    # Define the fitness function to minimize the total distance of the route
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm object
    ga = funsearch.GeneticAlgorithm(fitness, _distances.shape[0])

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route found
    return best_route
