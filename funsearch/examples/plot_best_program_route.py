import os
import util as util
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


n_cities_graph = 20
base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
target_dir = os.path.join(base_dir, 'tests', f'tsp-{n_cities_graph}')
filename = "280854846b3f43a38f7fcdd2fd7ba335.pkl"
file_path = os.path.join(target_dir, filename)
data = util.load_pkl_from_test(file_path)
print(data.matrix_distances[2,9])
print(data.matrix_distances[5,17])
lp_aco_route = list(map(int, data.route[:-1]))
print(lp_aco_route)

llm_route = [12, 1, 6, 19, 0, 11, 9, 14, 13, 15, 5, 4, 16, 18, 3, 10, 17, 7, 2, 8, 12]
print(llm_route)

cities = [coord for key, coord in sorted(data.cities.items(), key=lambda x: int(x[0]))]

print(data.cities)
print("LLM coordinates:")
for i in llm_route:
    print(i, cities[i])
print("LP-ACO coordinates:")
for i in lp_aco_route:
    print(i, cities[i])

def plot_routes(cities, route1, route2, label1='LLM', label2='LP-ACO'):
    x = [c[0] for c in cities]
    y = [c[1] for c in cities]
    show_name = True

    def plot_route(route, style, label):
        x_r = [cities[i][0] for i in route]
        y_r = [cities[i][1] for i in route]
        plt.plot(x_r, y_r, style, label=label)

        if show_name:
            # Label the cities if show_name is True
            print(route)
            for idx in route:
                plt.text(cities[idx][0], cities[idx][1], idx)

    plot_route(route1, 'b-', label1)
    plot_route(route2, 'r--', label2)
    plt.legend()
    plt.title("Comparación de rutas LLM vs LP-ACO")
    plt.axis('equal')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()


# main
plot_routes(cities, llm_route, lp_aco_route)
