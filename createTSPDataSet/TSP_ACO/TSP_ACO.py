import numpy as np
from matplotlib import pyplot as plt


class Ant:
    def __init__(self, start_city):
        self.start_city = start_city
        self.current_city = start_city
        self.visited_cities = [start_city]
        self.path_length = 0
        self.path = [start_city]

    def choose_next_city(self, n_cities, distances, pheromones, alpha, beta):
        unvisited_cities = [city for city in range(n_cities) if city not in self.visited_cities]
        probs = np.zeros(len(unvisited_cities))
        for i, city in enumerate(unvisited_cities):
            if distances[city, self.current_city] > 0:
                probs[i] = pheromones[city, self.current_city] ** alpha * \
                           (1 / distances[city, self.current_city]) ** beta
            else:
                probs[i] = 0
        probs /= np.sum(probs)
        # Choose the next city
        next_city = np.random.choice(unvisited_cities, p=probs)

        self.path_length += distances[self.current_city, next_city]
        self.current_city = next_city
        self.visited_cities.append(next_city)
        self.path.append(next_city)

def ant_system(n_cities, n_ants, n_iterations, distances, pheromones, alpha, beta, Q, rho):
    # Create ants
    ants = [Ant(i) for i in range(n_ants)]
    # Main loop
    for i in range(n_iterations):
        # Reset visited cities for each ant at the beginning of each iteration
        for ant in ants:
            ant.visited_cities = [ant.start_city]
            ant.current_city = ant.start_city
            ant.path_length = 0
            ant.path = [ant.start_city]

        # Ants build their paths
        for _ in range(n_cities-1):
            for ant in ants:
                ant.choose_next_city(n_cities, distances, pheromones, alpha, beta)

        # Add the starting city to the end of the path for each ant
        for ant in ants:
            ant.path_length += distances[ant.current_city, ant.start_city]
            ant.path.append(ant.start_city)

        # Update pheromones
        for i in range(n_cities):
            for j in range(i + 1, n_cities):
                pheromones[i, j] *= (1 - rho)
                pheromones[j, i] *= (1 - rho)

        for ant in ants:
            for i in range(len(ant.path) - 1):
                pheromones[ant.path[i], ant.path[i + 1]] += Q / ant.path_length
                pheromones[ant.path[i + 1], ant.path[i]] += Q / ant.path_length
    return ants

def plot_best_path(best_path, cities):
    plt.figure(figsize=(10, 6))

    coordinate_x = [cities[ciudad][0] for ciudad in cities]
    coordinate_y = [cities[ciudad][1] for ciudad in cities]

    plt.scatter(coordinate_x, coordinate_y, color="blue", label="Cities")

    # Graphic best path
    for i in range(len(best_path) - 1):
        x1, y1 = cities[best_path[i]]
        x2, y2 = cities[best_path[i + 1]]
        plt.plot([x1, x2], [y1, y2], color="red", linewidth=2, label="Best Path" if i == 0 else "")

    for ciudad in cities:
        x, y = cities[ciudad]
        plt.text(x + 1, y + 1, str(ciudad), fontsize=12, color="black")

    plt.title("Best Path Found by Ant Colony Algorithm")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    plt.grid()
    plt.show()