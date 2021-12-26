import random
import numpy as np
import math
import re
from itertools import combinations
import matplotlib.pyplot as plt


def calculate_dist(cords1, cords2):
    return math.sqrt((cords1[0] - cords2[0])**2 + (cords1[1] - cords2[1])**2)


def map_generator(available_nodes, density=0.5, weights_range=(0, 100)):
    """
    Gera uma lista de conexoes com pesos no intervalo 'weights_range'
    baseado numa lista de ids em 'available_nodes', a variavel 'density'
    especifica o quao conectados os nos vao estar.
    """
    map_data = set({})
    rand_cords = list(np.random.randint(weights_range[0], weights_range[1]+1, (len(available_nodes), 2)))
    random.shuffle(available_nodes)
    map_cords = list(map(lambda node_id, cords: (node_id, cords), available_nodes, rand_cords))
    connections = list(combinations(map_cords, 2))

    connections.sort(key=lambda connection: calculate_dist(connection[0][1], connection[1][1]))

    most_far_nodes = (connections[-1][0][0], connections[-1][1][0])

    for map_cord in map_cords:
        node_edges = list(filter(lambda connection: connection[0][0] == map_cord[0], connections))
        node_edges.sort(key=lambda connection: calculate_dist(map_cord[1], connection[1][1]))

        size = int(len(node_edges) * density)
        if size > len(node_edges):
            size = len(node_edges)
        elif size == 0:
            size = 1
        node_edges = node_edges[:size]

        for connection in node_edges:

            conn_dist = calculate_dist(connection[0][1], connection[1][1])
            noise = random.random() + 1
            map_data.add(
                (
                    (connection[0][0], tuple(connection[0][1])),
                    (connection[1][0], tuple(connection[1][1])),
                    round(conn_dist * noise, 3)
                )
            )

    print(f"Mapa gerado com {len(map_data)} conexoes")
    return map_data, most_far_nodes


def find_smaller(d, alg):
    if alg == 'a_star':
        return min(d.items(), key=lambda k: k[1][2])[0]
    if alg == 'greedy':
        return min(d, key=d.get)
    else:
        print('Invalid algorithm!')
        return None


def heuristic(vertex_a, vertex_b):
    return calculate_dist((vertex_a.vertex_x, vertex_a.vertex_y), (vertex_b.vertex_x, vertex_b.vertex_y))


def format_solution(solution: list):
    formated_solution = re.sub("\[|\]", "", str(solution))
    formated_solution = re.sub(" ", "", str(formated_solution))
    return re.sub(",", "->", formated_solution)


def save_metrics(filename, write_header=True, close_on_end=True, **metrics):

    output_file = open("./outputs/" + filename, "a")
    metric_header = list(metrics.keys())

    if write_header:
        joint = ", "
        output_file.write(f"{joint.join(metric_header)}\n")

    line = ""
    for metric in metric_header:
        line += f"{metrics[metric]}, "
    line = line[:-2] + "\n"

    output_file.write(line)

    if close_on_end:
        output_file.close()


def display_graph(graph, filename, id_size=7, scale_factor=1.00):

    plt.style.use("bmh")

    nodes = list(graph.graph.keys())

    fig = plt.Figure(figsize=(17,15))

    for node in nodes:
        plt.scatter([node.vertex_x], [node.vertex_y])
        node_id = plt.annotate(node.vertex_id, (node.vertex_x, node.vertex_y))
        node_id.set_fontsize(id_size)

        edges = list(graph[node])

        for edge in edges:
            plt.plot([node.vertex_x, edge.vertex_x], [node.vertex_y, edge.vertex_y], color="#7777AA", lw=0.5)

    xmin, xmax = plt.xlim()
    ymin, ymax = plt.ylim()

    plt.xlim(xmin * scale_factor, xmax * scale_factor)
    plt.ylim(ymin * scale_factor, ymax * scale_factor)

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.savefig("./outputs/" + filename)
    plt.clf()
