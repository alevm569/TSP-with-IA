def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with the first city
    route = [0]

    # Create a set to track visited cities
    visited = {0}

    # Perform local search until all cities are visited
    while len(visited) < len(_distances):
        # Find the nearest unvisited city to the last city in the route
        nearest_city = None
        min_distance = math.inf

        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[route[-1]][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route and mark it as visited
        route.append(nearest_city)
        visited.add(nearest_city)

    # Return the complete route
    return tuple(route)
