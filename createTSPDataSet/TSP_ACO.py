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
            probs[i] = pheromones[city, self.current_city] ** alpha * \
                       (1 / distances[city, self.current_city]) ** beta
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