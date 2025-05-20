def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with the following enhancements:

    - Uses a hybrid approach combining the nearest neighbor heuristic with the 2-opt local search algorithm.
    - Implements a constraint satisfaction framework to enforce the requirement that all cities are visited exactly once and return to the starting point.
    """

    # Initialize the constraint satisfaction framework.
    problem = funsearch.Problem()

    # Create variables representing the order in which cities are visited.
    cities = list(range(len(_distances)))
    variables = [problem.add_variable(city) for city in cities]

    # Add constraints to ensure that each city is visited exactly once.
    for city in cities:
        problem.add_constraint(funsearch.AllDifferentConstraint(variables))

    # Add constraints to ensure that the route returns to the starting city.
    problem.add_constraint(funsearch.CircuitConstraint(variables))

    # Define the objective function as the total route distance.
    def total_distance(assignment):
        route = [variable.get_value(assignment) for variable in variables]
        return calculate_route_distance(route, _distances)

    problem.set_objective(total_distance)

    # Use hybrid search with nearest neighbor and 2-opt local search.
    solver = funsearch.HybridSearch(
        funsearch.NearestNeighborHeuristic(), funsearch.TwoOptLocalSearch(), _distances
    )

    # Solve the problem using the constraint satisfaction framework.
    assignment = solver.solve(problem)

    # Return the best route found.
    return tuple(variable.get_value(assignment) for variable in variables)
