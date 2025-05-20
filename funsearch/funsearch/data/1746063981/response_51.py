def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    This version implements a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize the route with the first city
    route = [0]

    # Create a copy of the distances matrix to track visited cities
    distances = _distances.copy()
    distances[0] = np.inf

    # Find the nearest neighbor for the first city
    for _ in range(len(_distances) - 1):
        # Find the city with the minimum distance to the last city in the route
        nearest_city = np.argmin(distances[route[-1]])

        # Add the nearest city to the route and mark it as visited
        route.append(nearest_city)
        distances[route[-2]][nearest_city] = np.inf

    # Find the cheapest insertion for the last city
    for i in range(len(_distances)):
        if i not in route:
            # Calculate the distance from the last city in the route to city i
            distance = _distances[route[-1]][i]

            # If the distance is smaller than the current best distance, update the route
            if distance < distances[route[-1]][route[-2]]:
                route.append(i)

    return tuple(route)
