def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize variables
    num_cities = len(_distances)
    best_distance = np.inf
    best_route = None

    # Generate random starting point
    start_city = np.random.randint(num_cities)

    # Initialize current route and distance
    current_route = [start_city]
    current_distance = 0

    # Iterate until all cities are visited
    while len(current_route) < num_cities:
        # Get the last city in the current route
        last_city = current_route[-1]

        # Find the next city using a combination of nearest neighbor and cheapest insertion
        nearest_city = np.argmin(_distances[last_city])
        cheapest_city = current_route[np.argmin([_distances[last_city][c] for c in current_route])]

        # Choose the city that minimizes the distance to the last city and the overall route
        if _distances[last_city][nearest_city] < _distances[last_city][cheapest_city]:
            next_city = nearest_city
        else:
            next_city = cheapest_city

        # Add the next city to the current route and update the distance
        current_route.append(next_city)
        current_distance += _distances[last_city][next_city]

    # Return to the starting city
    current_route.append(start_city)
    current_distance += _distances[current_route[-2]][current_route[-1]]

    # Check if the current route is better than the best route found so far
    if current_distance < best_distance:
        best_distance = current_distance
        best_route = current_route

    # Return the best route
    return tuple(best_route)
