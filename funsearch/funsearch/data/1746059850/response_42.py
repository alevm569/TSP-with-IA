def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using local search."""

    # Initialize a random route
    current_route = np.random.permutation(np.arange(len(_distances)))

    # Perform local search by iteratively swapping two cities in the route
    for _ in range(1000):
        i, j = np.random.randint(0, len(_distances), 2)
        current_route[i], current_route[j] = current_route[j], current_route[i]

        # Calculate the distance of the new route
        new_distance = calculate_route_distance(current_route, _distances)

        # If the new route is shorter, keep it
        if new_distance < calculate_route_distance(current_route, _distances):
            current_route = new_distance

    return current_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
