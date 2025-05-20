def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform nearest neighbor search to initialize a solution
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        # Find the city with the shortest distance from the current city
        nearest_city = np.argmin(_distances[current_city])

        # Ensure the nearest city is not already in the route
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Close the route by returning to the starting city
    route.append(start_city)

    return tuple(route)
