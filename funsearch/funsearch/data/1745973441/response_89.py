def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform hybrid heuristic
    for _ in range(100):
        # Use nearest neighbor to find a promising candidate city
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route = np.append(route, nearest_city)

        # Use cheapest insertion to find the best city to add to the route
        best_city = None
        best_distance = math.inf
        for city in range(len(_distances)):
            if city not in route:
                distance = _distances[current_city][city]
                if distance < best_distance:
                    best_city = city
                    best_distance = distance

        # Add the best city to the route
        route = np.append(route, best_city)

    return route
