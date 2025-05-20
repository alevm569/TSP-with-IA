def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the current route
    current_route = list(range(len(_distances)))
    np.random.shuffle(current_route)

    # Perform local search using the 2-opt heuristic
    for _ in range(100):
        for i in range(len(current_route)):
            for j in range(i + 2, len(current_route)):
                # Swap two cities in the route
                current_route[i], current_route[j] = current_route[j], current_route[i]

                # Calculate the distance of the new route
                new_distance = calculate_route_distance(current_route, _distances)

                # If the new route is shorter, keep it
                if new_distance < calculate_route_distance(current_route, _distances):
                    current_route = current_route

    # Convert the list to a tuple
    return tuple(current_route)
