#pip install matplotlib networkx
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

# Coordenadas de los nodos en el plano 2D
coordinates = np.array([
    [0, 0],    # Nodo 0
    [10, 0],   # Nodo 1
    [10, 10],  # Nodo 2
    [0, 10],   # Nodo 3
])

def euclidean_distance(node1, node2):
    """
    Calcula la distancia euclidiana entre dos nodos dados sus índices.
    """
    return np.sqrt(np.sum((node1 - node2) ** 2))

def generate_distance_matrix(coordinates):
    """
    Genera la matriz de distancias de un grafo completamente conectado basado en las coordenadas de los nodos.
    """
    n = len(coordinates)
    distance_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(i + 1, n):
            distance = euclidean_distance(coordinates[i], coordinates[j])
            distance_matrix[i, j] = distance
            distance_matrix[j, i] = distance  # La matriz es simétrica

    return distance_matrix

def main(graph, start_node=0):
    """
    Solución del TSP usando solve y evaluando su calidad.
    """
    # Genera una solución usando la función `solve`
    solution = solve(graph, start_node)

    # Evalúa la calidad de la solución
    score = evaluate(solution, graph)
    print(f"Ruta generada: {solution}, Costo total: {score}")

    # Grafica la ruta óptima
    plot_path(solution, coordinates, graph)


def evaluate(route, graph):
    """
    Evalúa la calidad de una ruta (solución).
    Calcula el costo total de la ruta.
    Penaliza rutas incompletas o inválidas.
    """
    if len(set(route)) != len(graph) or len(route) != len(graph):
        # Penalización si no se visitan todos los nodos exactamente una vez
        return float('inf')  # Penalización alta para rutas inválidas

    # Calcula el costo total de la ruta
    cost = sum(graph[route[i - 1], route[i]] for i in range(len(route)))
    cost += graph[route[-1], route[0]]  # Regresa al nodo inicial
    return cost

def solve(graph, start_node=0):
    """
    Construye una solución al TSP utilizando la heurística `priority`.
    """
    n = len(graph)
    route = [start_node]  # Nodo inicial
    unvisited = set(range(n)) - {start_node}

    while unvisited:
        current_node = route[-1]
        # Selecciona el próximo nodo según la heurística
        next_node = max(unvisited, key=lambda x: priority(current_node, x, graph))
        route.append(next_node)
        unvisited.remove(next_node)

    return route

def priority(current_node, candidate_node, graph):
    """
    Calcula el puntaje de prioridad para moverse al nodo siguiente.
    Heurística inicial: favorece la distancia más corta.
    """
    return -graph[current_node][candidate_node]  # Prioriza menor distancia

def plot_graph(coordinates, graph):
    """
    Grafica el grafo completamente conectado.
    """
    G = nx.Graph()

    # Añadir nodos
    for i, coord in enumerate(coordinates):
        G.add_node(i, pos=tuple(coord))

    # Añadir las aristas con las distancias como pesos
    for i in range(len(coordinates)):
        for j in range(i + 1, len(coordinates)):
            G.add_edge(i, j, weight=graph[i][j])

    # Posiciones de los nodos para la visualización
    pos = {i: tuple(coordinates[i]) for i in range(len(coordinates))}

    # Dibujar el grafo
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=15, font_weight='bold', edge_color='gray')
    plt.title("Grafo Completamente Conectado")
    plt.show()

def plot_path(route, coordinates, graph):
    """
    Grafica la ruta óptima encontrada.
    """
    # Generar las coordenadas de los nodos visitados en la ruta
    path_coords = [coordinates[i] for i in route]
    path_coords.append(coordinates[route[0]])  # Regresar al nodo inicial

    # Dibujar el grafo original
    plot_graph(coordinates, graph)

    # Dibujar la ruta óptima
    path_coords = np.array(path_coords)
    plt.plot(path_coords[:, 0], path_coords[:, 1], 'o-', color='red', markersize=10, linewidth=2, label="Ruta óptima")
    plt.legend()
    plt.title("Ruta Óptima Encontrada")
    plt.show()

# Generar la matriz de distancias usando las coordenadas
graph = generate_distance_matrix(coordinates)

# Ejecutar el algoritmo y mostrar la ruta óptima
main(graph)