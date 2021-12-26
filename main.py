import pygame
import utils
from collections import namedtuple, OrderedDict
from graph import Graph
from tkinter import *
from tkinter import messagebox
import os
import time


bg_color = (255, 255, 255)
node_color = (30, 144, 255)
edge_color = (0, 0, 0)
node_shape = (10, 10)
grid_spacing = 50
nb_nodes = 8
bg_shape = (1300, 750)
screen = pygame.display.set_mode(bg_shape)


class GraphDraw(object):
    def __init__(self):
        self.nodes = set()

    def add(self, node):
        self.nodes.add(node)

    def get_node(self, index):
        for node in self.nodes:
            if node.id == index:
                return node
        return None

    def update(self):
        screen.fill(bg_color)
        for node in self.nodes:
            pygame.draw.rect(screen, node.color, node.rect)
            screen.blit(node.txt_surf, (node.rect.right + 10, node.rect.top))
            for neighbor in node.neighbors:
                pygame.draw.line(screen, edge_color, node.rect.center, neighbor.rect.center)


class Node(object):
    creation_counter = 0

    def __init__(self, x, y, index):
        # self.id = self.__class__.creation_counter
        # self.__class__.creation_counter += 1

        self.id = index
        self.x = x
        self.y = y

        self.font = pygame.font.Font(None, 20)
        self.font_color = (0, 0, 0)
        self.rect = pygame.Rect(self.x, self.y, node_shape[0], node_shape[1])
        # self.txt_surf = self.font.render(str(self.rect.topleft), True, self.font_color)
        self.txt_surf = self.font.render(str(self.id), True, self.font_color)
        self.color = node_color
        self.neighbors = set()

    def __hash__(self):
        return self.id


def search(nodes, index):
    for node in nodes:
        if node.id == index:
            return True
    return False


def create_graph():
    test = (30, 375, 0.2)
    graph = Graph()
    cities = list(range(test[0]))

    test_map, most_far_nodes = utils.map_generator(cities, test[2], weights_range=(-1 * test[1], test[1]))
    vertex = namedtuple("Vertex", ["vertex_id", "vertex_x", "vertex_y"])

    graph_draw = GraphDraw()
    nodes = []

    for connection in test_map:
        node1 = vertex(vertex_id=connection[0][0], vertex_x=connection[0][1][0], vertex_y=connection[0][1][1])
        node2 = vertex(vertex_id=connection[1][0], vertex_x=connection[1][1][0], vertex_y=connection[1][1][1])
        weight = connection[2]
        graph.add_edge(node1, node2, weight)

        if not search(nodes, index=node1):
            a = Node(x=node1.vertex_x + test[1], y=node1.vertex_y + test[1], index=node1.vertex_id)
            nodes.append(a)
            graph_draw.add(a)
        else:
            for i in nodes:
                if i.id == node1.vertex_id:
                    a = i
                    break
        if not search(nodes, index=node2):
            b = Node(x=node2.vertex_x + test[1], y=node2.vertex_y + test[1], index=node2.vertex_id)
            nodes.append(b)
            graph_draw.add(b)
        else:
            for i in nodes:
                if i.id == node2.vertex_id:
                    b = i
                    break

        a.neighbors.add(b)
        b.neighbors.add(a)

    return graph_draw, graph


def a_star(graph, start_id, end_id, graph_draw):
    expanded = 0
    branching_factor = []

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]


    openList = {}
    closedList = {}

    # A lista contém respectivamente: g(custo acumulado), funcao h da heuristica e funcao f(g + h)
    openList[start_node] = [0, utils.heuristic(
        start_node, end_node), 0 + utils.heuristic(start_node, end_node)]

    success = False
    solution = []
    parentMap = {}

    while openList:
        current = utils.find_smaller(openList, 'a_star')

        current_draw = graph_draw.get_node(index=current.vertex_id)
        pygame.draw.rect(screen, (124, 252, 0), current_draw.rect)
        pygame.display.update()
        time.sleep(1)

        closedList[current] = openList[current]
        del openList[current]
        if current == end_node:
            success = True
            break
        else:
            expanded += 1
            n = 0
            for child in graph[current]:
                if child in closedList:
                    continue
                if child in openList:
                    new_g = closedList[current][0] + graph[current][child].weight
                    if openList[child][0] > new_g:
                        openList[child][0] = new_g
                        openList[child][2] = new_g + openList[child][1]
                        parentMap[child] = current
                        n += 1
                else:
                    child_g = closedList[current][0] + graph[current][child].weight
                    child_h = utils.heuristic(child, end_node)
                    openList[child] = [child_g, child_h, child_g + child_h]
                    parentMap[child] = current
                    n += 1
            branching_factor.append(n)

    cost = 0
    if success:
        while current != start_node:
            solution.append(current.vertex_id)
            cost += graph[current][parentMap[current]].weight
            current = parentMap[current]
        solution.append(current.vertex_id)
        solution.reverse()

        depth = len(solution) - 1
        average = sum(branching_factor) / len(branching_factor)

        Tk().wm_withdraw()
        result = messagebox.askokcancel('Program Finished',
                                        ('The program finished, the shortest distance \n to the path is ' +
                                         str("{:.2f}".format(cost)) +
                                         ' blocks away, \n would you like to re run the program?'))
        if result:
            os.execl(sys.executable, sys.executable, *sys.argv)
            pygame.quit()

        return solution, depth, cost, expanded, len(openList) + len(closedList), average, 'success'
    else:
        return solution, -1, cost, expanded, len(openList), -1, 'failure'


def onsubmit():
    global start
    global end
    st = startBox.get()
    ed = endBox.get()
    start = int(st)
    end = int(ed)
    window.quit()
    window.destroy()


pygame.init()

window = Tk()
window.geometry('200x100')
label0 = Label(window, text='Start id: ')
# label0.place(x=0, y=100)
startBox = Entry(window)
# startBox.place(x=50, y=100)
label1 = Label(window, text='End id: ')
# label1.place(x=0, y=200)
endBox = Entry(window)
# endBox.place(x=50, y=200)
submit = Button(window, text='Submit', command=onsubmit)
# submit.place(x=50, y=300)
window.update()

# showPath.grid(columnspan=2, row=2)
submit.grid(columnspan=2, row=3)
label1.grid(row=1, pady=3)
endBox.grid(row=1, column=1, pady=3)
startBox.grid(row=0, column=1, pady=3)
label0.grid(row=0, pady=3)

graph_draw, graph = create_graph()

while True:
    ev = pygame.event.poll()
    if ev.type == pygame.QUIT:
        pygame.quit()
    graph_draw.update()
    pygame.display.update()
    window.mainloop()
    a_star(start_id=start, end_id=end, graph_draw=graph_draw, graph=graph)
    # pygame.display.flip()
    # pygame.time.delay(30)
