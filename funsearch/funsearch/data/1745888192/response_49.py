def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize starting point
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Create a copy of the distance matrix
    distances = _distances.copy()

    # Mark the starting city as visited
    distances[start_city, :] = np.inf
    distances[:, start_city] = np.inf

    # Iterate until all cities are visited
    while len(route) < len(_distances):

        # Cheapest insertion heuristic
        nearest_city = np.argmin(distances[current_city, :])
        route.append(nearest_city)

        # Update distances matrix
        distances[current_city, nearest_city] = np.inf
        distances[nearest_city, current_city] = np.inf

        # Update current city
        current_city = nearest_city

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
