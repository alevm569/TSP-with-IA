def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining two heuristics:
        - **Nearest neighbor:** Starts from an initial city and iteratively selects the closest unvisited city.
        - **Cheapest insertion:** Inserts the next city that minimizes the total route distance.

    Local search is also employed to refine the solution.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(_distances):
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city][[city for city in range(len(_distances)) if city not in visited]])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Refine route using cheapest insertion heuristic
    for i in range(len(route)):
        current_city = route[i]
        cheapest_city = np.argmin(_distances[current_city][route[(i + 1) % len(route)]])
        if cheapest_city != route[(i + 1) % len(route)]:
            route[i + 1] = cheapest_city

    # Perform local search to further optimize the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:]
            new_route[i + 1], new_route[j] = new_route[j], new_route[i + 1]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
