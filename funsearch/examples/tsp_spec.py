"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics
PROVIDE just the python code for the new version of the function, i.e. find_best_route_vx"""
import numpy as np
from numpy import ndarray
import funsearch

"""Distances matrix between cities"""
distances = np.array([
   [0, 10, 15, 20],
   [10, 0, 35, 25],
   [15, 35, 0, 30],
   [20, 25, 30, 0],
])

def calculate_route_distance(route: tuple[int, ...], _distances: ndarray) -> float:
  """function to calculate the total distance of a given route."""
  """sum the distances between cities in the route"""
  distance = sum(_distances[route[i], route[i + 1]] for i in range(len(route) - 1))
  """add the distance from the last city to the first city"""
  distance += _distances[route[-1], route[0]]
  return int(distance)
@funsearch.run
def evaluate(n: int) -> float:
  """Evaluate the find_best_route function, calculating the total distance of the best route."""
  """here the evaluation of the evolved function (respect the signature)"""
  best_route = find_best_route(distances)
  return calculate_route_distance(best_route, distances)


@funsearch.evolve
def find_best_route(_distances) -> tuple[int, ...]:
  """Find the best route to solve the TSP problem."""
  return tuple(range(len(_distances)))
