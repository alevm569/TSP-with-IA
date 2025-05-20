def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic, such as the ant colony optimization algorithm.
    # This heuristic should be more efficient than the previous version.

    # Example using ant colony optimization:
    from pywrapcp import RoutingIndexManager, PyRoutingModel

    num_cities = len(_distances)
    manager = RoutingIndexManager(num_cities, 1, 0)
    routing = PyRoutingModel(manager)

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert indices to nodes.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return _distances[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = routing.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Extract the route.
    route = [manager.IndexToNode(i) for i in assignment.Value(routing.StartVar(0))]
    route.append(route[0])  # Close the route.

    return tuple(route)
