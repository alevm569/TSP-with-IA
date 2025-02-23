"""Finds heuristics.

Add some heuristics to solve TSP problem
"""
import itertools
import logging
from typing import List
import numpy as np
import funsearch

# Coordenadas de los nodos en el plano 2D
coordinates = np.array([
    [0, 0],  # Nodo 0
    [10, 0],  # Nodo 1
    [10, 10],  # Nodo 2
    [0, 10],  # Nodo 3
])

"""Calcula la distancia euclidiana entre dos nodos dados sus índices."""


def euclidean_distance(node1: int, node2: int):
    return np.sqrt(np.sum((node1 - node2) ** 2))


""" Genera la matriz de distancias de un grafo completamente conectado basado en las coordenadas de los nodos."""


def generate_distance_matrix(coordinates) -> np.ndarray:
    n = len(coordinates)
    distance_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(i + 1, n):
            distance = euclidean_distance(coordinates[i], coordinates[j])
            distance_matrix[i, j] = distance
            distance_matrix[j, i] = distance  # La matriz es simétrica

    return distance_matrix


"""Calculates the total distance of a route."""


def calculate_distance_route(route: List[int], graph: np.ndarray):
    return sum(graph[route[i], route[i + 1]] for i in range(len(route) - 1))


@funsearch.memoize
def track_progress():
    return {"best_distance": float("inf"), "no_improvement_count": 0, "history": []}


"""
  Evalúa la calidad de una ruta (solución).
  Calcula el costo total de la ruta.
  Penaliza rutas incompletas o inválidas.
"""


@funsearch.run
def evaluate(n: int) -> float:
    state = track_progress()  # Obtener estado global

    graph = generate_distance_matrix(coordinates)
    route = solve(graph, 0)
    total_distance = calculate_distance_route(route, graph)

    # Actualizar la mejor distancia y el contador de iteraciones sin mejora
    if total_distance < state["best_distance"]:
        state["best_distance"] = total_distance
        state["no_improvement_count"] = 0  # Reiniciar contador
    else:
        state["no_improvement_count"] += 1

    state["history"].append(total_distance)
    if len(state["history"]) > 10:
        state["history"].pop(0)  # Mantener solo las últimas 10 iteraciones

    if state["no_improvement_count"] >= 10:
        logging.info("No hay mejoras en 10 iteraciones. Deteniendo FunSearch.")
        return float("inf")

    return total_distance


""" Construye una solución al TSP utilizando la heurística priority."""


def solve(graph: np.ndarray, start_node=0):
    n = len(graph)
    route = [start_node]  # Nodo inicial
    unvisited = set(range(n)) - {start_node}

    while unvisited:
        current_node = route[-1]
        # Selecciona el próximo nodo según la heurística
        next_node = min(unvisited, key=lambda x: priority(current_node, x, graph))
        route.append(next_node)
        unvisited.remove(next_node)

    return route


"""
Calcula el puntaje de prioridad para moverse al nodo siguiente.
Heurística inicial: favorece la distancia más corta.
"""


@funsearch.evolve
def priority(current_node, candidate_node, graph) -> float:
    return float("inf")
