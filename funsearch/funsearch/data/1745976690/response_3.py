def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with additional heuristic."""

    # Initialize variables
    num_cities = len(_distances)
    best_distance = float('inf')
    best_route = None

    # Generate initial random route
    random_route = np.random.permutation(num_cities)

    # Apply 2-opt heuristic
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            new_route = np.delete(random_route, [i, j])
            new_route = np.insert(new_route, i, new_route[j])
            new_route = np.insert(new_route, j, new_route[i])

            # Calculate distance of new route
            new_distance = calculate_route_distance(new_route, _distances)

            # Update best route if necessary
            if new_distance < best_distance:
                best_distance = new_distance
                best_route = new_route

    return best_route
