def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a combination of nearest neighbor and cheapest insertion.

    # Example heuristic:
    # 1. Start from the first city.
    # 2. Find the city with the shortest distance to the current city.
    # 3. Add the city to the route.
    # 4. Repeat steps 2-3 until all cities have been visited.
    # 5. Close the route by returning to the first city.

    # Return the best route as a tuple of integers.
    return tuple(np.random.permutation(len(_distances)))
