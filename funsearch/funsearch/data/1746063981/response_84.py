def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.

    # Example heuristic:
    # 1. Start from an arbitrary city.
    # 2. Find the city with the minimum distance to the current city.
    # 3. Add the city to the route and mark it as visited.
    # 4. Repeat steps 2-3 until all cities have been visited.
    # 5. Return to the starting city.

    # Example code for the heuristic:
    route = [0]  # Start from city 0
    visited = set([0])

    while len(visited) < len(_distances):
        current_city = route[-1]
        next_city = np.argmin([_distances[current_city][j] for j in range(len(_distances)) if j not in visited])
        route.append(next_city)
        visited.add(next_city)

    route.append(0)  # Return to starting city

    return tuple(route)
