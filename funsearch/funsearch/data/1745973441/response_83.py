def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a combination of local search and the nearest neighbor heuristic.
    # Local search will refine the route, while the nearest neighbor heuristic will guide the search.

    # Initialize the current route.
    current_route = list(range(len(_distances)))
    np.random.shuffle(current_route)

    # Run local search.
    for i in range(100):  # Number of iterations
        # Find the best neighbor route.
        best_neighbor = None
        best_distance = float('inf')

        for j in range(len(current_route)):
            for k in range(j + 1, len(current_route)):
                # Swap two cities in the route.
                new_route = current_route[:]
                new_route[j], new_route[k] = new_route[k], new_route[j]

                # Calculate the distance of the new route.
                distance = calculate_route_distance(new_route, _distances)

                # Update the best neighbor route.
                if distance < best_distance:
                    best_neighbor = new_route
                    best_distance = distance

        # If the best neighbor route is better, update the current route.
        if best_distance < calculate_route_distance(current_route, _distances):
            current_route = best_neighbor

    # Return the best route as a tuple of city indices.
    return tuple(current_route)
