def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Implement an improved heuristic or combine multiple heuristics to improve performance.
    Consider using a hybrid approach that combines different strategies.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Implement your new heuristic or combination of heuristics here.
    # For example, you could use a combination of nearest neighbor and cheapest insertion.

    # Return the best route as a tuple of city indices.
    return tuple(range(len(_distances)))
