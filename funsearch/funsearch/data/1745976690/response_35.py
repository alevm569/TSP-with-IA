def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combine multiple heuristics to improve performance.
    # Consider using a hybrid approach that combines different strategies.

    # Example heuristic:
    def nearest_neighbor(start: int) -> list[int]:
        visited = [start]
        current = start
        while len(visited) < len(_distances):
            min_distance = float('inf')
            for i in range(len(_distances)):
                if i not in visited and _distances[current][i] < min_distance:
                    min_distance = _distances[current][i]
                    next_city = i
            visited.append(next_city)
            current = next_city
        return visited

    # Use the nearest neighbor heuristic to generate an initial route.
    initial_route = nearest_neighbor(0)

    # Perform local search on the initial route to find a better solution.
    best_route = local_search(initial_route, _distances)

    # Return the best route as a tuple of city indices.
    return best_route

def local_search(route: list[int], _distances: np.ndarray) -> list[int]:
    """Performs local search on a given route."""

    # Implement a local search algorithm here, such as the 2-opt heuristic.

    # Return the best route found during local search.
    return route
