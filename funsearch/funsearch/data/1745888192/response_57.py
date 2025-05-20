def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Iteratively find the closest unvisited city and add it to the route
    while unvisited:
        current = route[-1]
        closest = min(unvisited, key=lambda c: _distances[current][c])
        route.append(closest)
        unvisited.remove(closest)

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
