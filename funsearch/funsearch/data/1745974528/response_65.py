def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a metaheuristic optimization algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can explore a large number of candidate routes and find the best one.

    # Example using simulated annealing:
    from funsearch.algorithms import simulated_annealing

    # Define the fitness function to minimize the total route distance.
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Run the simulated annealing algorithm.
    best_route = simulated_annealing(fitness_function, len(_distances), max_iterations=1000)

    # Return the best route found.
    return best_route
