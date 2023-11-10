from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np
import math
import uuid


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


# Agente cargador
class Cargador(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.carga = 100

    def posCargador(M, N, num_cuadrantes_x, num_cuadrantes_y, i, j):
        tamañoM = M // num_cuadrantes_x
        tamañoN = N // num_cuadrantes_y
        pos_x = tamañoM * i + tamañoM // 2
        pos_y = tamañoN * j + tamañoN // 2
        return pos_x, pos_y


class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.destination = (0,0) #variable que almacenará el cargador al que se debe dirigir si su batería llega a un nivel crítico

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos
    
    # cuando no tiene una celda sucia cerca se mueve de manera aleatoria (?)
    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        # return [vecino for vecino in lista_de_vecinos
        #                 if isinstance(vecino, Celda) and vecino.sucia]
        # #Opción 2
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias
    
    @staticmethod
    def distancia_euclidiana(punto1, punto2):
        x1, y1 = punto1
        x2, y2 = punto2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def buscar_cargador(self, origin):
        lista_tuplas = [(4, 4), (4, 14), (14, 4), (14, 14)] #cambiar por un arreglo de entrada para la lista de cargadores
        minDistance = float('inf')
        closestCharger = None

        x, y = origin
        print (x)
        print (y)
        # Iterar sobre la lista de tuplas
        for tupla in lista_tuplas:
            aux_x, aux_y = tupla
            distancia = self.distancia_euclidiana((x, y), (aux_x, aux_y))
            if abs(distancia) < abs(minDistance):
                minDistance = distancia
                closestCharger = tupla
        #Establecemos un destino para que el robot se dirija ahi en caso de llegar a un nivel de batería crítico
        d_x, d_y = closestCharger
        self.destination = (d_x, d_y)
        print(self.destination)

    def ir_a_cargador(self):
        print("LLENDO A CARGAR...")
        x_cargador, y_cargador = self.destination
        print(f"DESTINO: {self.destination}")
        x, y = self.pos
        if x < x_cargador:
            x = x + 1
        elif x > x_cargador:
            x = x - 1
        
        if y < y_cargador:
            y = y + 1
        elif y > y_cargador: 
            y = y - 1
                
        self.sig_pos = (x, y)
        print(f"SIGUIENTE PASO: {self.sig_pos}")

    
    def step(self):
        if self.carga > 30:
            vecinos = self.model.grid.get_neighbors(
                self.pos, moore=True, include_center=False)

            for vecino in vecinos:
                if isinstance(vecino, (Mueble, RobotLimpieza)):
                    vecinos.remove(vecino)

            celdas_sucias = self.buscar_celdas_sucia(vecinos)

            if len(celdas_sucias) == 0:
                self.seleccionar_nueva_pos(vecinos)
            else:
                self.limpiar_una_celda(celdas_sucias)
        else: 
            if self.destination == (0,0):
                print("BATTERY RUNNING OUT!")
                self.buscar_cargador(self.pos)
            else: 
                self.ir_a_cargador()


    def advance(self):
        # En caso de querer meter una negociación con otros agentes, se debería de colocar aqui
        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0:
            self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 num_cuadrantesX: int = 2, 
                 num_cuadrantesY: int = 2
                 ):

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles
        self.num_cuadrantesX = num_cuadrantesX
        self.num_cuadrantesY = num_cuadrantesY

        self.grid = MultiGrid(M, N, False) #multigrid permite que haya varios agentes en la misma celda 
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

        # Posicionamiento de cargadores
        for i in range(num_cuadrantesX):
            for j in range(num_cuadrantesY):
                pos_x, pos_y = Cargador.posCargador(M, N, num_cuadrantesX, num_cuadrantesY, i, j)
                cargador = Cargador(f"{i * num_cuadrantesY + j}", self)
                self.schedule.add(cargador)
                self.grid.place_agent(cargador, (pos_x, pos_y))

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
