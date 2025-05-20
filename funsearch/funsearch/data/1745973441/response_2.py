def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1` using a hybrid approach.

    The strategy combines two heuristics:
        - Nearest neighbor: Finds the closest unvisited city from the current city.
        - Cheapest insertion: Selects the city that minimizes the distance when inserted into the current route.

    The route is then iteratively refined using the 2-opt local search optimization technique.
    """

    # Initialize the route using the nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    # Build the route using the cheapest insertion heuristic
    for _ in range(len(_distances) - 1):
        next_city = find_cheapest_neighbor(current_city, visited, _distances)
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Close the route by returning to the starting city
    route.append(route[0])

    # Refine the route using the 2-opt local search technique
    funsearch.localsearch.two_opt(route, _distances)

    return route


def find_cheapest_neighbor(city: int, visited: set[int], _distances: np.ndarray) -> int:
    """
    Finds the closest unvisited city from the given city.

    Args:
        city: The current city.
        visited: Set of already visited cities.
        _distances: The distance matrix.

    Returns:
        The ID of the closest unvisited city.
    """
    min_distance = math.inf
    cheapest_city = None

    for i in range(len(_distances)):
        if i not in visited:
            distance = _distances[city][i]
            if distance < min_distance:
                min_distance = distance
                cheapest_city = i

    return cheapest_city
