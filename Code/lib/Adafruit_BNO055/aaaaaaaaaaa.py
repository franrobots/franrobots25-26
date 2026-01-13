import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import serial

bluetooth_port = 'COM4'


def bluetooth_value():
    return bluetooth.readline().decode().strip()
        
class GraphViewer:
    def __init__(self, matrix=np.zeros((81, 81)), rows: int=9, columns: int=9) -> None:
        if type(matrix) != 'numpy.ndarray':
            matrix = np.array(matrix)
        self.matrix = matrix
        self.rows = rows
        self.columns = columns
        self.n_nodes = rows * columns
        self.point_nodes = []
        self.actual_point = (0, 0)# (rows//2, columns//2)
        self.visit_nodes = [self.point_to_index(self.actual_point)]


    def set_actual_point(self, actualPoint: tuple):
        if 0 < actualPoint[0] < self.rows and 0 < actualPoint[1] < self.columns:
            self.actual_point = actualPoint
            self.visit_nodes.append(self.actual_point) if self.actual_point not in self.visit_nodes else None


    def draw_image(self):
        self.create_grid()
        self.draw_edges()
        self.draw_nodes()
        self.draw_indexes()


    def create_grid(self):
        scale = 100
        offset = scale//2
        self.size_img = (self.rows * scale, self.columns * scale, 3)
        self.img = np.ones((self.rows * scale, self.columns * scale, 3), np.int8) * 255
        for i in range(self.rows):
            for j in range(self.columns):
                point = (j * scale + offset, i * scale + offset)
                self.point_nodes.append(point)
    

    def draw_edges(self):
        for i in range(self.matrix.shape[0]):
            for j in range(i + 1, self.matrix.shape[1]):
                if self.matrix[i, j] != 0:
                    cv2.line(self.img, self.point_nodes[i], self.point_nodes[j], (0, 0, 0), 2)
                    self.visit_nodes.append(i) if i not in self.visit_nodes else None
                    self.visit_nodes.append(j) if j not in self.visit_nodes else None


    def draw_indexes(self):
        nodes_index = 0
        for i in range(self.n_nodes):
            cv2.putText(self.img, f'{nodes_index}', (self.point_nodes[nodes_index][0]-  8, self.point_nodes[nodes_index][1]+7),cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            nodes_index += 1


    def draw_nodes(self):
        for i in range(self.n_nodes):
            cv2.circle(self.img, self.point_nodes[i], 33, (0, 0, 0), -1)
            color = (200, 200, 200) if i not in self.visit_nodes else (0, 255, 0)
            if i == self.point_to_index(self.actual_point):
                color = (255, 0, 0)
            cv2.circle(self.img, self.point_nodes[i], 30, color, -1)

    def draw_info(self):
        space = 40
        init = 30
        path, _, instructions = self.getMininumPathBetween(self.point_to_index(self.actual_point), self.visit_nodes[0])
        texts = [f'Best Path (from {self.point_to_index(self.actual_point)} to {self.visit_nodes[0]}):',
                 f'{path}',
                 f'Best Path Instructions:',
                 f'{instructions}']
        textSize = 1
        for i, text in enumerate(texts):
            textSize = 1 if i != 3 else 0.7
            cv2.putText(self.controller, text, (20, init + i*space), cv2.FONT_HERSHEY_COMPLEX, textSize, (0, 0, 0), 1)
        cv2.putText(self.controller, f'Actual point: {self.actual_point} | {self.point_to_index(self.actual_point)}', (20, self.size - 15), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 1)

    def draw_controller(self):
        self.size = 1000
        self.controller = np.ones((self.size, self.size, 3)) * 255
        self.controllerSize = 60
        self.upController = [(self.size//2 - self.controllerSize, self.size//2 - self.size//5 - self.controllerSize),
                             (self.size//2 + self.controllerSize, self.size//2 - self.size//5 + self.controllerSize)]
        self.downController = [(self.size//2 - self.controllerSize, self.size//2 + self.size//5 - self.controllerSize),
                               (self.size//2 + self.controllerSize, self.size//2 + self.size//5 + self.controllerSize)]

        self.leftController = [(self.size//2 - self.size//5 - self.controllerSize, self.size//2 - self.controllerSize),
                               (self.size//2 - self.size//5 + self.controllerSize, self.size//2 + self.controllerSize)]
        self.rightController = [(self.size//2 + self.size//5 - self.controllerSize, self.size//2 - self.controllerSize),
                                (self.size//2 + self.size//5 + self.controllerSize, self.size//2 + self.controllerSize)]
        color = (0, 0, 0)
        cv2.rectangle(self.controller, self.upController[0],self.upController[1], color, -1)
        cv2.rectangle(self.controller, self.downController[0],self.downController[1], color, -1)
        cv2.rectangle(self.controller, self.leftController[0],self.leftController[1], color, -1)
        cv2.rectangle(self.controller, self.rightController[0],self.rightController[1], color, -1)


    def start(self):
        self.fig = plt.figure("Graph Visualizer and Controller")
        self.draw_image()
        self.draw_controller()
        self.draw_info()
        self.ax1 = plt.subplot(1, 2, 1)
        self.img1 = plt.imshow(self.img)
        plt.axis('off')
        self.ax2 = plt.subplot(1, 2, 2)
        self.img2 = plt.imshow(self.controller)
        plt.axis('off')
        plt.connect('button_press_event', self.on_click)
        plt.ion()
        plt.show(block=True)
        plt.ioff()

    
    def start_by_bluetooth(self):
        self.fig = plt.figure("Graph Visualizer and Controller")
        self.draw_image()
        self.draw_controller()
        self.draw_info()
        self.ax1 = plt.subplot(1, 2, 1)
        self.img1 = plt.imshow(self.img)
        plt.axis('off')
        self.ax2 = plt.subplot(1, 2, 2)
        self.img2 = plt.imshow(self.controller)
        plt.axis('off')
        self.fig.canvas.mpl_connect('close_event', self.close)
        plt.ion()
        while plt.isinteractive():
            if bluetooth.in_waiting > 0:
               resposta = bluetooth_value()
               print(resposta)
               self.update_matrix(int(resposta))
               if resposta == 'q':
                   break
            plt.pause(0.001)
        plt.ioff()


    def close(self):
        print('oi')
        plt.ioff()
        plt.close()
        exit()


    def show(self):
        self.draw_image()
        plt.figure("Graph Visualizer")
        plt.imshow(self.img)
        plt.axis('off')
        plt.show()


    def on_click(self, event):
        if event.inaxes == self.ax2:
            x, y = event.xdata, event.ydata
            controllers = [self.upController, self.downController, self.leftController, self.rightController]
            commands = ['up', 'down', 'left', 'right']
            for i, controller in enumerate(controllers):
                if controller[0][0] <= x <= controller[1][0] and controller[0][1] <= y <= controller[1][1]:
                    self.updateByDirection(commands[i])
                    self.draw_image()
                    self.draw_controller()
                    self.draw_info()
                    self.img1.set_data(self.img)
                    self.img2.set_data(self.controller)


    def updateByDirection(self, direction: str):
        actualIndex = self.point_to_index(self.actual_point)
        if direction == 'up':
            if self.actual_point[0] > 0:
                nextIndex = self.point_to_index((self.actual_point[0] - 1, self.actual_point[1]))
                self.matrix[actualIndex, nextIndex] = 1
                self.matrix[nextIndex, actualIndex] = 1
                self.actual_point = self.index_to_point(nextIndex)
                self.visit_nodes.append(nextIndex) if nextIndex not in self.visit_nodes else None
                print(f"Info: Move {direction} successfull. Actual point: {self.actual_point}")
            else:
                print(f'Error: Move {direction} out of bounds. Actual point: {self.actual_point}')
        elif direction == 'down':
            if self.actual_point[0] < self.rows - 1:
                nextIndex = self.point_to_index((self.actual_point[0] + 1, self.actual_point[1]))
                self.matrix[actualIndex, nextIndex] = 1
                self.matrix[nextIndex, actualIndex] = 1
                self.actual_point = self.index_to_point(nextIndex)
                self.visit_nodes.append(nextIndex) if nextIndex not in self.visit_nodes else None
                print(f"Info: Move {direction} successfull. Actual point: {self.actual_point}")
            else:
                print(f'Error: Move {direction} out of bounds. Actual point: {self.actual_point}')
        elif direction == 'left':
            if self.actual_point[1] > 0:
                nextIndex = self.point_to_index((self.actual_point[0], self.actual_point[1] - 1))
                self.matrix[actualIndex, nextIndex] = 1
                self.matrix[nextIndex, actualIndex] = 1
                self.actual_point = self.index_to_point(nextIndex)
                self.visit_nodes.append(nextIndex) if nextIndex not in self.visit_nodes else None
                print(f"Info: Move {direction} successfull. Actual point: {self.actual_point}")
            else:
                print(f'Error: Move {direction} out of bounds. Actual point: {self.actual_point}')
        elif direction == 'right':
            if self.actual_point[1] < self.columns - 1:
                nextIndex = self.point_to_index((self.actual_point[0], self.actual_point[1] + 1))
                self.matrix[actualIndex, nextIndex] = 1
                self.matrix[nextIndex, actualIndex] = 1
                self.actual_point = self.index_to_point(nextIndex)
                self.visit_nodes.append(nextIndex) if nextIndex not in self.visit_nodes else None
                print(f"Info: Move {direction} successfull. Actual point: {self.actual_point}")
            else:
                print(f'Error: Move {direction} out of bounds. Actual point: {self.actual_point}')


    def update_matrix(self, index):
        if 0 <= index < self.n_nodes:
            self.matrix[self.point_to_index(self.actual_point), index] = 1
            self.matrix[index, self.point_to_index(self.actual_point)] = 1
            self.actual_point = self.index_to_point(index)
            self.draw_image()
            self.draw_controller()
            self.draw_info()
            self.img1.set_data(self.img)
            self.img2.set_data(self.controller)
        else:
            print(f'Error: Update Matrix index {index} out of bounds.')


    def index_to_point(self, index) -> tuple:
        row = index//self.columns
        return (row, index - row*self.columns)
    

    def point_to_index(self, point) -> int:
        return point[0]*self.columns + point[1]


    def move(self, direction: str):
        direction = direction.lower()
        if direction in ['up', 'down', 'left', 'right']:
            self.updateByDirection(direction.lower())
        else:
            print(f'Direction \'{direction}\' is not valid! Actual Point = {self.actual_point}')


    
    def adjacency(self, p) -> list:
        adjacency_list = []
        for j in range(len(self.matrix)):
            if self.matrix[p, j] != 0:
                adjacency_list.append(j)
        return adjacency_list


    def bfs(self, p1) -> tuple:
        fila, pai, d, cor = [], [], [], []
        for _ in range(len(self.matrix)):
            pai.append(-1)
            d.append(float('inf'))
            cor.append(1)
        cor[p1] = 0
        d[p1] = 0
        pai[p1] = -1
        fila.append(p1)
        while len(fila) > 0:
            u = fila[0]
            del fila[0]
            for i in self.adjacency(u):
                if cor[i] == 1:
                    cor[i] = 0
                    d[i] = d[u] + 1
                    pai[i] = u
                    fila.append(i)
        return (d, pai)


    def getMininumPathBetween(self, index_1, index_2) -> tuple:
        if isinstance(index_1, tuple):
            index_1 = self.point_to_index(index_1)
            index_2 = self.point_to_index(index_2)
        _, pai = self.bfs(index_2)
        path = []
        path.append(index_1)
        while pai[index_1] != -1:
            path.append(pai[index_1])
            index_1 = pai[index_1]
        coordinates = [self.index_to_point(i) for i in path]
        return (path, coordinates, self.getInstructions(coordinates))
    

    def getInstructions(self, coordinates) -> list:
        commands = []
        for i in range(len(coordinates) - 1):
            row_dif = coordinates[i+1][0] - coordinates[i][0]
            if row_dif != 0:
                command = 'up' if row_dif < 0 else 'down'
            else:
                col_dif = coordinates[i+1][1] - coordinates[i][1]
                command = 'left' if col_dif < 0 else 'right'
            commands.append(command)
        return commands


row = 9
col = 9

matrix = np.zeros((row*col, row*col))
for i in [(40, 41), (41, 42), (42, 51), (51, 60), (60, 59), (59, 58)]:
    matrix[i[0], i[1]] = 1
    matrix[i[1], i[0]] = 1



#matrix = [[0, 1, 0, 0, 0, 0], [1, 0, 1, 0, 1, 0], [0, 1, 0, 0, 0, 1], [0, 0, 0, 0, 1, 0], [0, 1, 0, 1, 0, 1], [0, 0, 1, 0, 1, 0]]
try:
    bluetooth = serial.Serial(bluetooth_port, 9600)  
    print("Conexão Bluetooth estabelecida com sucesso.")
except serial.SerialException:
    print("Erro ao conectar ao dispositivo Bluetooth. Verifique se o dispositivo está emparelhado e conectado.")
    exit()
graph = GraphViewer()
graph.start_by_bluetooth()