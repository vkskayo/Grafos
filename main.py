# Importa a classe Graph do módulo onde está implementada (separadamente ou no mesmo arquivo)
from graph_final import Graph  # Descomente se o Graph estiver em um módulo separado

def main():
    # Carrega o grafo a partir de um arquivo
    graph = Graph(num_vertices=5, weighted=True)  # Inicialização com 0 vértices, será substituído pelo arquivo
    graph.load_from_file('graph_data.txt')  # Substitua pelo nome do seu arquivo

    # Exibe informações básicas do grafo
    print("Informações do grafo:")
    print(f"Vértices: {graph.num_vertices}")
    print("Lista de adjacência:")
    for vertex, neighbors in graph.adj_list.items():
        print(f"{vertex}: {neighbors}")

    # Executa a BFS a partir do vértice 0
    start_vertex = 0
    print(f"\nExecutando BFS a partir do vértice {start_vertex}:")
    parent_bfs, level_bfs = graph.bfs(start_vertex)
    print(f"Pais: {parent_bfs}")
    print(f"Níveis: {level_bfs}")

    # Executa a DFS a partir do vértice 0
    print(f"\nExecutando DFS a partir do vértice {start_vertex}:")
    parent_dfs, level_dfs = graph.dfs(start_vertex)
    print(f"Pais: {parent_dfs}")
    print(f"Níveis: {level_dfs}")

    # Encontra o caminho mais curto entre dois vértices
    end_vertex = graph.num_vertices - 1  # Exemplo: até o último vértice
    print(f"\nEncontrando o caminho mais curto de {start_vertex} a {end_vertex}:")
    path, distance = graph.shortest_path(start_vertex, end_vertex)
    if path:
        print(f"Caminho mais curto: {path} com distância {distance}")
    else:
        print(f"Não há caminho entre {start_vertex} e {end_vertex}.")

    # Salva informações do grafo em um arquivo de saída
    graph.save_graph_info('graph_info_output.txt')
    print("\nInformações do grafo foram salvas em 'graph_info_output.txt'.")

if __name__ == "__main__":
    main()
