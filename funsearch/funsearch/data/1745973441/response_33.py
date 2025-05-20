def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform hybrid heuristic
    for _ in range(100):
        # Use nearest neighbor heuristic to find a new city to add to the route
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city])
        route = np.append(route, nearest_city)

        # Use cheapest insertion heuristic to find the best position to insert the new city
        best_position = np.argmin([calculate_route_distance(route[:i] + [nearest_city] + route[i:], _distances) for i in range(len(route) + 1)])
        route = np.insert(route, best_position, nearest_city)

    return route
