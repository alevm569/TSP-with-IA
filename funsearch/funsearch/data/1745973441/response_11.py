def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform local search using 2-opt heuristic
    for _ in range(100):
        # Randomly select two different cities in the route
        i, j = np.random.randint(0, len(_distances), 2)

        # Calculate the distance of the original route
        original_distance = calculate_route_distance(route, _distances)

        # Perform the 2-opt swap
        route[i], route[j] = route[j], route[i]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(route, _distances)

        # If the new route is shorter, keep the swap
        if new_distance < original_distance:
            pass
        else:
            # If the new route is longer, reverse the swap
            route[i], route[j] = route[j], route[i]

    return route
