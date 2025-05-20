def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform a local search using the 2-opt heuristic.
    current_route = np.random.permutation(len(_distances))
    best_distance = calculate_route_distance(current_route, _distances)

    while True:
        improved = False

        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                # Swap two cities in the route.
                new_route = current_route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the distance of the new route.
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    current_route = new_route
                    improved = True

        if not improved:
            break

    return current_route
