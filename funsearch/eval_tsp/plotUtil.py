import pickle

from matplotlib import pyplot as plt
import os

from eval_tsp.TSPSolution import TSPSolution
from eval_tsp.constants import Cities, Distances, Edges
from eval_tsp.distanceUtil import calculate_path_distance


def plot_route(cities: Cities, distances: Distances, route, show_name=True, title=None, marked_edges: Edges=None):
    if None in route:
        print("The route is not complete, and has None values")
        return
    # Get coordinates of the cities in the route
    coordinate_x = [cities[ciudad][0] for ciudad in route]
    coordinate_y = [cities[ciudad][1] for ciudad in route]

    # Plot to show the cities
    plt.figure(figsize=(8, 6))
    plt.scatter(coordinate_x, coordinate_y, color='blue', label='Cities')

    # Plot of the best route
    plt.plot(coordinate_x, coordinate_y, linestyle='-', marker='o', color='red', label='Best Route')
    if marked_edges is not None:
        for edge_from in marked_edges:
            edge_to = marked_edges[edge_from]
            plt.plot([cities[edge_from][0], cities[edge_to][0]], [cities[edge_from][1], cities[edge_to][1]],
                     linestyle='-', color='green', label=None, linewidth=5)

    if show_name:
        # Label the cities if show_name is True
        for i, ciudad in enumerate(route):
            plt.text(coordinate_x[i], coordinate_y[i], ciudad)

    # calculate the total distance of the route
    path_distance = calculate_path_distance(distances, route)
    plt.xlabel('Coordinate X')
    plt.ylabel('Coordinate Y')
    title = title if title is not None else 'Cities'
    title = title + ' (Distance: {:.4f})'.format(path_distance)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()


def read_solution_from_pickle(file_path: str) -> TSPSolution | None:
    if os.path.exists(file_path) is False:
        return None

    class CustomUnpickler(pickle.Unpickler):
        def find_class(self, module, name):
            if module == "TSPSolution":
                module = "eval_tsp.TSPSolution"
            return super().find_class(module, name)

    with open(file_path, 'rb') as f:
        solution = CustomUnpickler(f).load()
    return solution
