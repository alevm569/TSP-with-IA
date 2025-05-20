def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Incorporates a hybrid heuristic that combines two-opt and a local search algorithm.
    """

    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Two-opt heuristic to improve initial solution
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            route = two_opt(route, i, j)

    # Local search to refine solution
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                improved_route = two_opt(route, i, j)
                if calculate_route_distance(improved_route, _distances) < calculate_route_distance(route, _distances):
                    route = improved_route

    return route
