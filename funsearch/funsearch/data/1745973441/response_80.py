def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a combination of local search and a new heuristic.
    """

    # Define a new heuristic that prioritizes routes with smaller total distances.
    def distance_heuristic(route: tuple[int, ...]) -> float:
        total_distance = calculate_route_distance(route, _distances)
        return 1 / total_distance

    # Use a local search algorithm with the new heuristic.
    best_route = funsearch.local_search(
        initial_solution=tuple(range(len(_distances))),
        distance_function=distance_heuristic,
        neighborhood_operator=funsearch.swap_neighborhood,
        max_iterations=1000
    )

    return best_route
