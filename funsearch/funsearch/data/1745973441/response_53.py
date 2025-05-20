def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform hybrid heuristic
    for _ in range(100):
        # Apply 2-opt heuristic to improve local search
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(_distances)):
            for j in range(i + 1, len(_distances)):
                new_route = route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance

        # Perform local search
        for _ in range(10):
            # Randomly swap two cities in the route
            i, j = np.random.randint(0, len(_distances), 2)
            route[i], route[j] = route[j], route[i]

            # Check if the new route is better
            if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
                pass
            else:
                # If not, reverse the swap
                route[i], route[j] = route[j], route[i]

    return route
