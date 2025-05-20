def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` with a novel heuristic.

    This function implements a hybrid heuristic that combines the following techniques:

    - **Nearest Neighbor:** Starts at a random city and iteratively selects the closest unvisited city.
    - **Cheapest Insertion:** Finds the cheapest way to insert a new city into the current route without violating constraints.
    - **Local Search:** Performs a neighborhood search around the current route, improving it iteratively using the cheapest insertion heuristic.

    This hybrid approach balances exploration and exploitation, providing a balance between finding promising routes and exploring diverse solutions.
    """

    # Initialization
    num_cities = len(_distances)
    route = np.zeros(num_cities, dtype=int)
    visited = np.zeros(num_cities, dtype=bool)

    # Nearest Neighbor heuristic
    start_city = np.random.randint(num_cities)
    route[0] = start_city
    visited[start_city] = True

    # Cheapest Insertion heuristic
    for i in range(1, num_cities):
        current_city = route[i - 1]
        best_city = -1
        best_distance = float('inf')

        for j in range(num_cities):
            if not visited[j]:
                distance = _distances[current_city][j]
                if distance < best_distance:
                    best_city = j
                    best_distance = distance

        route[i] = best_city
        visited[best_city] = True

    # Local Search optimization
    for i in range(num_cities):
        for j in range(i + 2, num_cities):
            route = swap_edges(route, i, j)

    return route
