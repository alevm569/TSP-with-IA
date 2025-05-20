def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid approach."""

    # Perform nearest neighbor to initialize a partial route
    start_city = 0
    route = [start_city]
    unvisited = set(range(len(_distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Use 2-opt to improve the route quality
    def two_opt(route):
        best_route = route[:]
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:]
                new_route[i:j+1] = new_route[j:i:-1]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                    best_route = new_route
        return best_route

    route = two_opt(route)

    return route
