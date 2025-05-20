def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Use a metaheuristic optimization algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can help find optimal or near-optimal solutions to the TSP problem.

    # Use a brute force algorithm with constraints.
    # This method guarantees finding the optimal solution if the number of cities is small enough.
    from constraint import Problem, AllDifferentConstraint

    # Create a constraint satisfaction problem.
    problem = Problem()

    # Add variables for each city.
    cities = list(range(len(_distances)))
    problem.addVariables(cities, cities)

    # Add constraint that each city is visited exactly once.
    problem.addConstraint(AllDifferentConstraint(), cities)

    # Add constraint that the route is connected (starts and ends at the same city).
    problem.addConstraint(lambda *route: route[0] == route[-1], cities)

    # Solve the problem using constraint satisfaction.
    solutions = problem.getSolutions()

    # Return the best route as a tuple of city indices.
    best_route = min(solutions, key=lambda solution: calculate_route_distance(solution, _distances))
    return best_route
