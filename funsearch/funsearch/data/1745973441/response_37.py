def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using the 2-opt heuristic."""

    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Apply the 2-opt heuristic repeatedly
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                # Swap two cities in the route
                new_route = route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the total distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is shorter, update the current route
                if new_distance < calculate_route_distance(route, _distances):
                    route = new_route

    # Return the best route found
    return route
