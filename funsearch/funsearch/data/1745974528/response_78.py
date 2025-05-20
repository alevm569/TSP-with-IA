def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the best route and distance
    best_route = None
    best_distance = float('inf')

    # Generate all possible permutations of the cities
    permutations = list(itertools.permutations(range(len(_distances))))

    # Iterate through all permutations
    for permutation in permutations:
        # Calculate the total distance of the route
        distance = 0
        for i in range(len(permutation)):
            distance += _distances[permutation[i]][permutation[(i + 1) % len(permutation)]]

        # Update the best route if necessary
        if distance < best_distance:
            best_distance = distance
            best_route = permutation

    return best_route
