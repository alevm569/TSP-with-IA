def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Iterate until all cities are visited
    while unvisited:
        # Get the last city in the route
        current_city = route[-1]

        # Find the closest unvisited city
        best_distance = float('inf')
        best_city = None
        for city in unvisited:
            distance = _distances[current_city][city]
            if distance < best_distance:
                best_distance = distance
                best_city = city

        # Add the closest city to the route and remove it from the unvisited set
        route.append(best_city)
        unvisited.remove(best_city)

    # Return the route, including the return to the starting city
    return tuple(route + [route[0]])
