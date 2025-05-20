def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize best route and distance
    best_route = tuple(range(len(_distances)))
    best_distance = calculate_route_distance(best_route, _distances)

    # Perform local search using 2-opt heuristic
    for _ in range(100):
        # Generate a random permutation of the route
        random_route = np.random.permutation(range(len(_distances)))

        # Apply 2-opt heuristic to find a better route
        for i in range(len(random_route)):
            for j in range(i + 1, len(random_route)):
                new_route = random_route[:i] + random_route[j:i:-1] + random_route[j + 1:]

                # Calculate distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is better, update the best route
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

    return best_route
