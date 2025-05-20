def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the 2-opt heuristic to find a better initial solution.
    best_route = find_best_route_v1(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(len(best_route)):
        for j in range(i + 1, len(best_route)):
            # Swap two cities in the route.
            new_route = best_route[:i] + (best_route[j],) + best_route[i+1:j] + (best_route[i],) + best_route[j+1:]
            new_distance = calculate_route_distance(new_route, _distances)

            # If the new route is shorter, update the best route.
            if new_distance < best_distance:
                best_route = new_route
                best_distance = new_distance

    return best_route
