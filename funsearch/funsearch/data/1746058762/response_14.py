def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using local search."""

    # Initialize the route with a random permutation of cities
    num_cities = len(_distances)
    route = np.random.permutation(num_cities)

    # Local search: iteratively improve the route by swapping two cities
    for _ in range(1000):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                # Calculate the distance difference if we swap the two cities
                distance_diff = _distances[route[i]][route[j]] + _distances[route[j]][route[(j + 1) % num_cities]] - _distances[route[i]][route[(i + 1) % num_cities]]

                # If swapping the two cities improves the route, do it
                if distance_diff < 0:
                    route[i], route[j] = route[j], route[i]

    return route
