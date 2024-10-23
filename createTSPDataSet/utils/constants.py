from enum import Enum


class Heuristics(str, Enum):
    NearestNeighbour = "nearest_neighbor"
    BestEdges = "best_edges"

# Typing
Cities = dict[str, tuple[float, float]]
Distances  = dict[tuple[str, str], float]
EdgeList = list[tuple[str, str]]
Edges = dict[str, str]