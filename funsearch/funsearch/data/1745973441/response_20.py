def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid approach that combines multiple heuristics, such as:

    # 1. Nearest neighbor heuristic to generate an initial candidate route.
    # 2. Cheapest insertion heuristic to refine the candidate route.
    # 3. Local search algorithm to optimize the route by iteratively swapping two cities.

    # Perform multiple iterations of the hybrid approach and return the route with the lowest total distance.
    best_route = None
    best_distance = float('inf')

    for _ in range(10):  # Adjust the number of iterations as needed
        # Run the hybrid approach to generate a candidate route
        candidate_route = hybrid_heuristic(_distances)

        # Calculate the distance of the candidate route
        candidate_distance = calculate_route_distance(candidate_route, _distances)

        # Update the best route if necessary
        if candidate_distance < best_distance:
            best_distance = candidate_distance
            best_route = candidate_route

    return best_route


def hybrid_heuristic(_distances: np.ndarray) -> tuple[int, ...]:
    # Implement the hybrid heuristic that combines multiple heuristics.
    pass
