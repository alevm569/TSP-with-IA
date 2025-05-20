def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic or strategy here.
    # For example, you could use a genetic algorithm or a hybrid of different heuristics.
    # Your goal is to improve upon the performance of find_best_route_v2.

    # Example heuristic:
    # Use a nearest neighbor strategy to initialize a route, then iteratively add the closest city not yet visited.
    route = [0]
    visited = {0}
    n = len(_distances)

    while len(visited) < n:
        current_city = route[-1]
        closest_city = None
        min_distance = float('inf')

        for i in range(n):
            if i not in visited and _distances[current_city][i] < min_distance:
                closest_city = i
                min_distance = _distances[current_city][i]

        route.append(closest_city)
        visited.add(closest_city)

    return tuple(route)
