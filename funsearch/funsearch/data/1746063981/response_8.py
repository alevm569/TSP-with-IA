def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a hybrid heuristic combining the nearest neighbor and cheapest insertion algorithms
    # Initialize the route with the nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    while len(route) < len(_distances):
        # Find the city with the shortest distance from the current city
        nearest_city = np.argmin(_distances[current_city])
        # Ensure the nearest city has not been visited yet
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Use the cheapest insertion heuristic to refine the route
    for i in range(len(_distances)):
        # Find the city that can be inserted into the route at the lowest cost
        best_city = None
        best_cost = float('inf')
        for j in range(len(_distances)):
            if j not in route:
                cost = _distances[route[-1]][j]
                if cost < best_cost:
                    best_city = j
                    best_cost = cost
        # Insert the best city into the route
        route.append(best_city)

    return tuple(route)
