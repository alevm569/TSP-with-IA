import math
import random
import string
from typing import List
from createTSPDataSet.utils.constants import Distances, Cities


def generate_cities(n_cities: int):
    cities = {}
    for i in range(n_cities):
        ciudad = get_different_city_name(list(cities.keys()))
        x = round(random.uniform(-100, 100) ,1) # Coordinate x random between -100 and 100
        y = round(random.uniform(-100, 100), 1)  # Coordinate y random between -100 and 100
        cities[ciudad] = (x, y)
    return cities

def get_different_city_name(cities: List[str]):
    city = f"{random.choice(string.ascii_uppercase)}{random.randint(0,9)}"
    while city in cities:
        city = f"{random.choice(string.ascii_uppercase)}{random.randint(0,9)}"
    return city

def get_distance(ciudad1, ciudad2):
    x1, y1 = ciudad1
    x2, y2 = ciudad2
    distancia = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distancia

def generate_distances(cities) -> Distances:
    distances = {}
    for ciudad1, coord1 in cities.items():
        for ciudad2, coord2 in cities.items():
            if ciudad1 != ciudad2:
                distancia = get_distance(coord1, coord2)
                distances[(str(ciudad1), str(ciudad2))] = distancia
    return distances

def generate_cities_with_distances(n_cities: int, seed: int =123) -> (Cities, Distances):
    random.seed(seed)  # This fixes the seed for reproducibility
    cities = generate_cities(n_cities)
    distances = generate_distances(cities)
    return cities, distances