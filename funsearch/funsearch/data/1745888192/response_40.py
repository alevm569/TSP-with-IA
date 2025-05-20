def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use the 2-opt heuristic for local search optimization.
    current_route = list(range(len(_distances)))
    best_distance = calculate_route_distance(current_route, _distances)

    for _ in range(100):  # Run local search for up to 100 iterations
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                # Swap two cities in the route
                new_route = current_route[:]
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is shorter, update the best route
                if new_distance < best_distance:
                    best_distance = new_distance
                    current_route = new_route

    # Return the best route as a tuple of city indices.
    return tuple(current_route)
