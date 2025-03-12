"""Resolver el problem de TSP avoid brute force approaches, implement some heuristics """
import numpy as np
from numpy import ndarray

import funsearch

# Matriz de distancias entre ciudades
distances = np.array([
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0],
])





@funsearch.run
def evaluate(n: int) -> float:
  """Encuantra la mejor ruta usando la funcion de prioridad."""

  def calculate_route_distance(route: tuple[int, ...], _distances: ndarray) -> float:
      """Calcula la distancia total de una ruta."""
      # Sumar las distancias de todas las ciudades en el orden dado por la ruta
      distance = sum(_distances[route[i], route[i + 1]] for i in range(len(route) - 1))
      # Añadir la distancia para regresar a la ciudad inicial (cierre del tour)
      distance += _distances[route[-1], route[0]]
      return int(distance)

  best_route = find_best_route(distances)
  return -calculate_route_distance(best_route, distances)  # Minimizar distancia


@funsearch.evolve
def find_best_route(_distances) -> tuple[int, ...]:
  """Define la prioridad basada en la longitud de la ruta."""
  return tuple(range(len(_distances)))

