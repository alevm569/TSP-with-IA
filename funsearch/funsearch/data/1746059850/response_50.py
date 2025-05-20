def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize variables
    num_cities = len(_distances)
    current_city = 0
    route = [current_city]

    # Generate random starting points
    random_starts = np.random.randint(num_cities, size=num_cities)

    # Iterate until all cities have been visited
    while len(route) < num_cities:
        # Find the closest unvisited city using the nearest neighbor heuristic
        closest_city = np.argmin(_distances[current_city][:])
        if closest_city not in route:
            route.append(closest_city)

        # Swap two random cities using the 2-opt heuristic
        if np.random.rand() < 0.5:
            i, j = np.random.randint(num_cities, size=2)
            route[i], route[j] = route[j], route[i]

        # Update the current city
        current_city = route[-1]

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
