def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a local search algorithm to find the best route.
    # Start with an initial random route and iteratively swap two cities to improve the distance.
    route = np.random.permutation(np.arange(len(_distances)))
    best_distance = calculate_route_distance(route, _distances)

    while True:
        # Iterate over all pairs of cities in the route.
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                # Swap the two cities in the route.
                route[i], route[j] = route[j], route[i]

                # Calculate the distance of the new route.
                distance = calculate_route_distance(route, _distances)

                # If the new route is better, keep it.
                if distance < best_distance:
                    best_distance = distance

                # Otherwise, restore the original route.
                route[i], route[j] = route[j], route[i]

        # If no improvements were made in an iteration, break.
        if best_distance == calculate_route_distance(route, _distances):
            break

    return route
