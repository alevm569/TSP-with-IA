def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize best route and distance
    best_route = list(range(len(_distances)))
    best_distance = calculate_route_distance(best_route, _distances)

    # Perform local search
    for i in range(len(best_route)):
        for j in range(i + 1, len(best_route)):
            # Swap two cities in the route
            best_route[i], best_route[j] = best_route[j], best_route[i]

            # Calculate distance of the new route
            distance = calculate_route_distance(best_route, _distances)

            # If the new route is better, update best route and distance
            if distance < best_distance:
                best_distance = distance

    # Return the best route
    return tuple(best_route)
