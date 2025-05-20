def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform nearest neighbor search to initialize a partial route.
    current_city = 0
    partial_route = [current_city]
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        partial_route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt optimization to improve the route.
    def two_opt(route):
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                reversed_route = route[:i] + route[i:j][::-1] + route[j:]
                distance = calculate_route_distance(reversed_route, _distances)
                if distance < best_distance:
                    best_distance = distance
                    best_route = reversed_route
        return best_route

    optimized_route = two_opt(partial_route)

    return optimized_route
