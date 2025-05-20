def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use a combination of local search and 2-opt heuristics
    best_route = find_best_route_v2(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        # Perform local search to find a better route
        improved_route = local_search(best_route, _distances)

        # Perform 2-opt to improve the route
        improved_route = two_opt(improved_route, _distances)

        # Calculate the distance of the improved route
        improved_distance = calculate_route_distance(improved_route, _distances)

        # If the improved route is better, update the best route and distance
        if improved_distance < best_distance:
            best_route = improved_route
            best_distance = improved_distance
        else:
            # If no further improvement is possible, return the best route found
            return best_route
