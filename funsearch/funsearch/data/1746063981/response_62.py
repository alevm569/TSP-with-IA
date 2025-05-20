def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize variables
    num_cities = len(_distances)
    best_distance = float('inf')
    best_route = None

    # Generate all possible permutations of cities
    for permutation in itertools.permutations(range(num_cities)):

        # Calculate the distance of the current route
        distance = 0
        for i in range(num_cities):
            distance += _distances[permutation[i]][permutation[(i + 1) % num_cities]]

        # Update best route if necessary
        if distance < best_distance:
            best_distance = distance
            best_route = permutation

    return best_route
