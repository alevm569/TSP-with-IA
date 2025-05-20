def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Create a set of candidate routes using a nearest neighbor heuristic.
    candidate_routes = []
    start_city = 0
    current_city = start_city
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        candidate_routes.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Apply a 2-opt heuristic to improve the candidate routes.
    best_route = candidate_routes
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(len(candidate_routes)):
        for j in range(i + 1, len(candidate_routes)):
            reversed_route = candidate_routes[:i] + candidate_routes[i:j][::-1] + candidate_routes[j:]
            distance = calculate_route_distance(reversed_route, _distances)

            if distance < best_distance:
                best_route = reversed_route
                best_distance = distance

    return best_route
