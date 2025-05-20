def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a combination of the nearest neighbor and cheapest insertion heuristics.

    # Example heuristic:
    # 1. Start at an arbitrary city.
    # 2. Find the city with the shortest distance to the current city.
    # 3. Add the next city to the route.
    # 4. Repeat step 2-3 until all cities have been visited.
    # 5. Return to the starting city.

    # Example implementation:
    route = [0]
    visited = set([0])
    for _ in range(len(_distances) - 1):
        next_city = np.argmin(_distances[route[-1]][[c for c in range(len(_distances)) if c not in visited]])
        route.append(next_city)
        visited.add(next_city)
    route.append(0)

    return tuple(route)
