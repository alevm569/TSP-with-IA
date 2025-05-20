def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement your new heuristic here.
    # You may use existing functions from the `funsearch` library or implement your own.

    # For example, you could use the nearest neighbor heuristic:
    current_city = 0
    route = []
    while len(route) < len(_distances):
        nearest_city = find_nearest_city(current_city, _distances)
        route.append(nearest_city)
        current_city = nearest_city

    return tuple(route)

def find_nearest_city(city: int, _distances: np.ndarray) -> int:
    """Finds the nearest city to the given city."""
    distances = _distances[city]
    return np.argmin(distances[np.delete(np.arange(len(distances)), city)])
