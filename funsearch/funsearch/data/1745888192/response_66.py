def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform local search using the 2-opt heuristic
    best_route = find_best_route_v2(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(100):  # Number of local search iterations
        for j in range(len(best_route)):
            for k in range(j + 2, len(best_route)):
                new_route = best_route[:j] + best_route[j+k:] + best_route[j+k:j+k+1] + best_route[j:j+k]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

    return best_route
