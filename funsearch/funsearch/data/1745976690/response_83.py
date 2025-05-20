def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Hybrid Heuristic:

    - **Initial Route:** Use the nearest neighbor heuristic to generate an initial route.
    - **Route Optimization:** Apply the 2-opt heuristic iteratively to improve the route by swapping two consecutive city pairs.

    This approach combines the strengths of both heuristics, providing a more efficient route-finding solution.
    """

    # Generate an initial route using the nearest neighbor heuristic.
    initial_route = funsearch.nearest_neighbor(_distances)

    # Optimize the route using the 2-opt heuristic.
    best_route = funsearch.two_opt(initial_route, _distances)

    return best_route
