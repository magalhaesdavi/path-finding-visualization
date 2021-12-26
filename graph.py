from collections import defaultdict, namedtuple


class Graph:
    """
    Classe de grafo baseado em lista de adjacencia.
    O grafo e um defaultdict de dict,
    no dict interior temos o vertice conectado com o valor do peso da aresta.
    """

    def __init__(self):
        self.graph = defaultdict(dict)

    def __len__(self):
        return len(self.graph)

    def __getitem__(self, key):
        if key not in self.graph:
            raise KeyError("Key {} not in the graph.".format(key))
        return self.graph[key]

    def __iter__(self):
        return self.graph.items()

    def __contains__(self, value):
        return value in self.graph

    def __str__(self):
        return self.plotGraph()

    def insertNode(self, node_id):
        """
        Insere um nó de id 'node_id' no grafo.
        A operação é cancelada caso o nó já exista.
        """
        if self.searchNode(node_id):
            return "ERROR! Node already in the graph"
        else:
            self.graph[node_id] = {}
            return True

    def add_edge(self, u, v, weight_=0, edgeID_=-1):
        """
        Conecta dois nos de id 'u' e 'v' atraves de uma arestas com um peso 'weight' associado.
        Caso os nos nao existam, os mesmos sao criados.
        O paramêtro 'inplace' especifica se, caso um dos nós não exista, os mesmos sejam criados.
        """

        if u not in self.graph:
            self.graph[u] = {}

        edge = namedtuple("Edge", ["edgeID", "weight"])
        self.graph[u][v] = edge(edgeID=edgeID_, weight=weight_)

        if v not in self.graph:
            self.graph[v] = {}
        self.graph[v][u] = edge(edgeID=edgeID_, weight=weight_)

        return True


    def plotGraph(self):
        plot = "######################\n"
        node_list = list(self.graph.items())
        node_list.sort(key=lambda node: node[0].vertex_id)

        for node in node_list:
            plot += f"{node[0].vertex_id} -> "
            for edge in node[1].items():
                plot += f"({edge[0].vertex_id}, {edge[1].weight})"

                if edge != list(node[1].items())[-1]:
                    plot += ','

            plot += "\n"
        plot += "######################\n"
        return plot