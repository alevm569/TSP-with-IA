def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or hybrid of heuristics here.
    # Consider using genetic algorithms, metaheuristics, or other advanced techniques.

    # Example of a hybrid heuristic:
    # 1. Use a nearest neighbor heuristic to find an initial route.
    # 2. Use a 2-opt heuristic to improve the route by swapping two consecutive edges.
    # 3. Repeat step 2 until no further improvements are found.

    # Ensure that the returned route satisfies all TSP constraints:
    # - Includes all cities exactly once.
    # - Returns to the starting city.

    # Perform local search to optimize the route
    current_route = find_best_route_v1(_distances)
    improved = True

    while improved:
        improved = False
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                # Swap two consecutive edges in the route
                new_route = current_route[:i] + current_route[j:i:-1] + current_route[j + 1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(current_route, _distances):
                    current_route = new_route
                    improved = True

    return current_route
