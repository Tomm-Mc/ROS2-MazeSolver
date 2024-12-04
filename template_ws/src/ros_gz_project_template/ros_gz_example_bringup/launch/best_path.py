import math
import cv2
import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, pos: tuple, g_cost: float, h_cost: float, parent=None):
        self.pos = pos  # (x, y) tuple
        self.g_cost = g_cost  # Coste G
        self.h_cost = h_cost  # Coste H (heurística)
        self.f_cost = self.g_cost + self.h_cost  # Coste F (G + H)
        self.parent = parent  # Nodo padre

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class AStar:
    def __init__(self, map_grid, margin=10):
        self.map_grid = map_grid  # Mapa de la grilla
        self.height, self.width = map_grid.shape
        self.margin = margin  # Margen de seguridad alrededor de las paredes

    def heuristic(self, node, goal):
        # Heurística de Manhattan
        return abs(node.pos[0] - goal.pos[0]) + abs(node.pos[1] - goal.pos[1])

    def get_neighbors(self, node):
        dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Movimientos: abajo, derecha, arriba, izquierda
        neighbors = []

        for dir in dirs:
            neighbor_pos = (node.pos[0] + dir[0], node.pos[1] + dir[1])

            if 0 <= neighbor_pos[0] < self.height and 0 <= neighbor_pos[1] < self.width:
                if self.is_safe(neighbor_pos):  # Verifica si la celda es segura
                    neighbors.append(Node(neighbor_pos, 0, 0))

        return neighbors

    def is_safe(self, pos):
        # Verificar si la celda está suficientemente alejada de las paredes
        x, y = pos

        # Revisar los 10 píxeles alrededor de la celda
        for i in range(x - self.margin, x + self.margin + 1):
            for j in range(y - self.margin, y + self.margin + 1):
                if 0 <= i < self.height and 0 <= j < self.width:
                    if self.map_grid[i, j] == 0:  # Si está cerca de una pared (0)
                        return False
        return True

    def search(self, start_pos, goal_pos):
        start_node = Node(start_pos, 0, self.heuristic(Node(start_pos, 0, 0), Node(goal_pos, 0, 0)))  
        goal_node = Node(goal_pos, 0, 0)  

        open_list = [start_node]  
        closed_list = set()  

        while open_list:
            open_list.sort()
            current_node = open_list.pop(0)

            if current_node.pos == goal_node.pos:
                return self.reconstruct_path(current_node)

            closed_list.add(current_node.pos)

            neighbors = self.get_neighbors(current_node)
            for neighbor in neighbors:
                if neighbor.pos in closed_list:
                    continue

                g_cost = current_node.g_cost + 1  
                h_cost = self.heuristic(neighbor, goal_node)
                f_cost = g_cost + h_cost

                if any(open_node.pos == neighbor.pos and open_node.f_cost <= f_cost for open_node in open_list):
                    continue

                neighbor.g_cost = g_cost
                neighbor.h_cost = h_cost
                neighbor.f_cost = f_cost
                neighbor.parent = current_node

                open_list.append(neighbor)

        return None  

    def reconstruct_path(self, goal_node):
        path = []
        current = goal_node

        while current:
            path.append(current.pos)
            current = current.parent

        return path[::-1]  # Devolver el camino en orden desde el inicio al objetivo

def pgm_to_grid(pgm_path, lower_threshold=0, upper_threshold=254):
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"El archivo {pgm_path} no fue encontrado.")
    
    plt.imshow(img, cmap='gray', origin='lower')
    plt.title("Imagen Original")
    plt.show()

    # Convertimos la imagen a una grilla donde 0 = no transitable y 255 = transitable
    grid = np.zeros_like(img, dtype=np.uint8)
    grid[img >= upper_threshold] = 255  # Celda transitable
    grid[img < lower_threshold] = 0  # Celda no transitable (pared)

    plt.imshow(grid, cmap='gray', origin='lower')
    plt.title(f"Grilla Generada con umbrales {lower_threshold}-{upper_threshold}")
    plt.show()

    return grid

if __name__ == "__main__":
    pgm_path = "/root/template_ws/Mapa.pgm"  
    grid = pgm_to_grid(pgm_path, lower_threshold=0, upper_threshold=254)

    # Coordenadas de inicio y objetivo
    # Obj Final = -15, 19 -> (-15+15.7) * 27 = 20, (19-21) * -18 = 36
    # Inicio = 0, 0 -> (0+15.7) * 27 = 425, (0-21) * -18 = 378
    start = (400, 350)
    goal = (50, 20)

    if start[0] >= grid.shape[0] or start[1] >= grid.shape[1] or \
       goal[0] >= grid.shape[0] or goal[1] >= grid.shape[1]:
        raise ValueError("Las coordenadas de inicio o fin están fuera del mapa.")

    # Ejecutar A*
    astar = AStar(grid, margin=10)
    path = astar.search(start, goal)

    if path:
        # Crear una copia del mapa para mostrar el camino
        path_grid = np.copy(grid)
        path_grid = cv2.cvtColor(path_grid, cv2.COLOR_GRAY2RGB)  # Convertir a RGB

        # Dibujar el camino (en rojo)
        for r, c in path:
            if 0 <= r < path_grid.shape[0] and 0 <= c < path_grid.shape[1]:
                path_grid[r, c] = [255, 0, 0]  # Camino en rojo

        plt.imshow(path_grid, cmap='gray', origin='lower')
        plt.title("Camino Encontrado en la Grilla")
        plt.show()

    else:
        print("No se encontró un camino entre los puntos especificados.")