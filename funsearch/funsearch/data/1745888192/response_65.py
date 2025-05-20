def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize variables
    num_cities = len(_distances)
    best_route = None
    best_distance = float('inf')

    # Generate all possible permutations of cities
    for perm in itertools.permutations(range(num_cities)):

        # Calculate the total distance of the route
        distance = 0
        for i in range(num_cities):
            distance += _distances[perm[i]][perm[(i + 1) % num_cities]]

        # Update best route if necessary
        if distance < best_distance:
            best_distance = distance
            best_route = perm

    return best_route
