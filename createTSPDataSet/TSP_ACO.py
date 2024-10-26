import random
import numpy as np
from typing import Dict, List, Tuple

class Ant:
    def __init__(self, cities, distances, alpha=1.0, beta=2.0):
        self.cities = cities
        self.distances = distances
        self.alpha = alpha  # Influencia de feromonas
        self.beta = beta    # Influencia heurística
        self.route = []
        self.total_distance = 0

    def choose_next_city(self, current_city, visited, pheromone):
        probabilities = []
        for city in self.cities:
            if city not in visited:
                tau = pheromone[(current_city, city)] ** self.alpha
                eta = (1 / self.distances[current_city, city]) ** self.beta
                probabilities.append(tau * eta)
            else:
                probabilities.append(0)

        probabilities = np.array(probabilities) / sum(probabilities)
        return np.random.choice(self.cities, p=probabilities)

    def build_route(self, pheromone):
        visited = set()
        current_city = random.choice(list(self.cities.keys()))
        self.route = [current_city]
        visited.add(current_city)

        while len(visited) < len(self.cities):
            next_city = self.choose_next_city(current_city, visited, pheromone)
            self.route.append(next_city)
            self.total_distance += self.distances[current_city, next_city]
            visited.add(next_city)
            current_city = next_city

class ACO:
    def __init__(self, cities, distances, n_ants=10, n_iterations=100, evaporation=0.5):
        self.cities = cities
        self.distances = distances
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.evaporation = evaporation
        self.pheromone = {(i, j): 1 for i in cities for j in cities if i != j}

    def run(self):
        best_ant = None
        for _ in range(self.n_iterations):
            ants = [Ant(self.cities, self.distances) for _ in range(self.n_ants)]
            for ant in ants:
                ant.build_route(self.pheromone)
            self.update_pheromone(ants)
            best_ant = min(ants, key=lambda x: x.total_distance)
        return best_ant.route, best_ant.total_distance

    def update_pheromone(self, ants: List[Ant]):
        for (i, j) in self.pheromone:
            self.pheromone[(i, j)] *= (1 - self.evaporation)  # Evaporación de feromonas

        for ant in ants:
            for i in range(len(ant.route) - 1):
                self.pheromone[(ant.route[i], ant.route[i + 1])] += 1 / ant.total_distance