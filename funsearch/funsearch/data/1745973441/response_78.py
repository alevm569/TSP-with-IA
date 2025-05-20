def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Use a metaheuristic optimization algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can help find optimal or near-optimal solutions to the TSP problem.

    # Example of using simulated annealing:
    from funsearch.algorithms import simulated_annealing

    # Define the initial route as a list of city indices.
    initial_route = list(range(len(_distances)))

    # Create a simulated annealing solver object.
    solver = simulated_annealing.SimulatedAnnealingSolver(_distances)

    # Run the solver to find the best route.
    best_route = solver.solve(initial_route)

    # Convert the best route to a tuple of city indices.
    return tuple(best_route)
