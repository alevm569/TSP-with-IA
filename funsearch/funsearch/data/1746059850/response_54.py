def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This function combines the nearest neighbor heuristic for initialization and the 2-opt local search heuristic for optimization.
    """

    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Apply nearest neighbor heuristic to fill in missing cities
    for i in range(len(route)):
        if route[i] is None:
            nearest_city = np.argmin(_distances[route[i - 1]])
            route[i] = nearest_city

    # Perform 2-opt local search to optimize the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route.copy()
                new_route[i:j+1] = route[j:i:-1]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    return route
