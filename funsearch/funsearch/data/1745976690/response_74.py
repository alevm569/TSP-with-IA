def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a hybrid heuristic combining the nearest neighbor and cheapest insertion algorithms.
    """

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Find the nearest neighbor for the first city
    current = 0
    while unvisited:
        nearest = min(unvisited, key=lambda city: _distances[current][city])
        route.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    # Perform cheapest insertion for the remaining unvisited cities
    while unvisited:
        cheapest = min(unvisited, key=lambda city: _distances[route[-1]][city])
        route.append(cheapest)
        unvisited.remove(cheapest)

    # Return the route to the starting city
    route.append(route[0])
    return tuple(route)
