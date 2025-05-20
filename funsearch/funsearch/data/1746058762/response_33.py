def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Perform nearest neighbor search to find an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt local search to improve the route.
    best_route = route[:]
    improved = True

    while improved:
        improved = False

        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                new_route = route[:]
                new_route[i:j] = route[j-1:i-1:-1]

                if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                    best_route = new_route
                    improved = True

    return best_route
