import heapq

class Graph:
    def __init__(self, num_vertices, directed=False, weighted=False):
        """
        Inicializa um grafo com a opção de ser direcionado e/ou ponderado.
        """
        self.num_vertices = num_vertices
        self.directed = directed
        self.weighted = weighted
        self.adj_list = {i: [] for i in range(num_vertices)}
        self.adj_matrix = [[0] * num_vertices for _ in range(num_vertices)]  # Matriz de adjacência opcional

    def add_edge(self, u, v, weight=1):
        """
        Adiciona uma aresta entre os vértices u e v.
        Se o grafo for direcionado, adiciona apenas na direção u -> v.
        Se o grafo não for ponderado, o peso padrão será 1.
        """
        self.adj_list[u].append((v, weight))
        self.adj_matrix[u][v] = weight

        if not self.directed:
            self.adj_list[v].append((u, weight))
            self.adj_matrix[v][u] = weight

    def bfs(self, start_vertex):
        """
        Implementa busca em largura (BFS) para grafos sem pesos.
        Retorna o vetor de pais e o nível de cada vértice.
        """
        visited = [False] * self.num_vertices
        parent = [-1] * self.num_vertices
        level = [-1] * self.num_vertices

        queue = [start_vertex]
        visited[start_vertex] = True
        level[start_vertex] = 0

        while queue:
            u = queue.pop(0)
            for (v, _) in self.adj_list[u]:
                if not visited[v]:
                    visited[v] = True
                    parent[v] = u
                    level[v] = level[u] + 1
                    queue.append(v)

        return parent, level

    def dijkstra(self, start_vertex):
        """
        Implementa o algoritmo de Dijkstra para encontrar o menor caminho em grafos ponderados.
        Retorna as distâncias e o vetor de pais.
        """
        distances = {v: float('inf') for v in range(self.num_vertices)}
        parent = {v: None for v in range(self.num_vertices)}
        distances[start_vertex] = 0

        min_heap = [(0, start_vertex)]

        while min_heap:
            current_distance, current_vertex = heapq.heappop(min_heap)

            if current_distance > distances[current_vertex]:
                continue

            for neighbor, weight in self.adj_list[current_vertex]:
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    parent[neighbor] = current_vertex
                    heapq.heappush(min_heap, (distance, neighbor))

        return distances, parent

    def shortest_path(self, start_vertex, end_vertex):
        """
        Encontra o caminho mais curto entre dois vértices.
        Usa BFS para grafos sem pesos e Dijkstra para grafos com pesos.
        """
        if self.weighted:
            distances, parent = self.dijkstra(start_vertex)
        else:
            parent, distances = self.bfs(start_vertex)

        # Reconstruir o caminho
        path = []
        current_vertex = end_vertex
        while current_vertex is not None:
            path.insert(0, current_vertex)
            current_vertex = parent[current_vertex]

        return path, distances[end_vertex]

    def all_shortest_paths(self, start_vertex):
        """
        Encontra o caminho mais curto de um vértice para todos os outros vértices.
        Usa BFS para grafos sem pesos e Dijkstra para grafos ponderados.
        """
        if self.weighted:
            distances, parent = self.dijkstra(start_vertex)
        else:
            parent, distances = self.bfs(start_vertex)

        paths = {}
        for vertex in range(self.num_vertices):
            if distances[vertex] != float('inf'):
                path = []
                current_vertex = vertex
                while current_vertex is not None:
                    path.insert(0, current_vertex)
                    current_vertex = parent[current_vertex]
                paths[vertex] = path

        return paths, distances

    def dfs_util(self, v, visited, parent, level, current_level):
        """
        Função auxiliar para DFS. Realiza a recursão.
        """
        visited[v] = True
        level[v] = current_level
        for (neighbor, _) in self.adj_list[v]:
            if not visited[neighbor]:
                parent[neighbor] = v
                self.dfs_util(neighbor, visited, parent, level, current_level + 1)

    def dfs(self, start_vertex):
        """
        Implementa busca em profundidade (DFS) para grafos.
        Retorna o vetor de pais e o nível de cada vértice.
        """
        visited = [False] * self.num_vertices
        parent = [-1] * self.num_vertices
        level = [-1] * self.num_vertices
        self.dfs_util(start_vertex, visited, parent, level, 0)
        return parent, level

    def find_connected_components(self):
        """
        Encontra todos os componentes conectados no grafo.
        """
        visited = [False] * self.num_vertices
        components = []
        for v in range(self.num_vertices):
            if not visited[v]:
                component = []
                self.dfs_component(v, visited, component)
                components.append(component)
        return components

    def dfs_component(self, v, visited, component):
        """
        Função auxiliar para encontrar componentes conectados usando DFS.
        """
        visited[v] = True
        component.append(v)
        for (neighbor, _) in self.adj_list[v]:
            if not visited[neighbor]:
                self.dfs_component(neighbor, visited, component)

    def load_from_file(self, filename):
        """
        Carrega um grafo a partir de um arquivo com a opção de pesos.
        """
        with open(filename, 'r') as file:
            lines = file.readlines()
            vertices = int(lines[0].strip())
            self.num_vertices = vertices
            self.adj_list = {i: [] for i in range(vertices)}

            for line in lines[1:]:
                edge_data = list(map(float, line.strip().split()))
                u, v = int(edge_data[0]), int(edge_data[1])
                weight = edge_data[2] if len(edge_data) > 2 else 1
                self.add_edge(u, v, weight)

    def save_graph_info(self, filename):
        """
        Salva as informações sobre o grafo em um arquivo.
        """
        num_edges = sum(len(neighbors) for neighbors in self.adj_list.values()) // (2 if not self.directed else 1)
        avg_degree = sum(len(neighbors) for neighbors in self.adj_list.values()) / self.num_vertices
        degree_distribution = [len(neighbors) for neighbors in self.adj_list.values()]

        with open(filename, 'w') as file:
            file.write(f"Número de vértices: {self.num_vertices}\n")
            file.write(f"Número de arestas: {num_edges}\n")
            file.write(f"Grau médio: {avg_degree:.2f}\n")
            file.write(f"Distribuição dos graus: {degree_distribution}\n")
