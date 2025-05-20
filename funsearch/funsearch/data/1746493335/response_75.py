def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use ACO heuristic to initialize route
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities)
    best_route = aco.run()

    # Apply 2-opt heuristic to improve route quality
    for i in range(len(best_route)):
        for j in range(i + 2, len(best_route)):
            distance_before = _distances[best_route[i]][best_route[j]]
            best_route[i+1:j] = reversed(best_route[i+1:j])
            distance_after = _distances[best_route[i]][best_route[j]]
            if distance_after < distance_before:
                break

    return tuple(best_route)
