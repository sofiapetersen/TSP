import os
import time
from itertools import permutations

def load_adjacency_matrix(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Arquivo {file_path} não encontrado.")
    with open(file_path, 'r') as f:
        matrix = [list(map(int, line.strip().split())) for line in f]
    return matrix

def calculate_path_cost(matrix, path):
    return sum(matrix[path[i]][path[i + 1]] for i in range(len(path) - 1)) + matrix[path[-1]][path[0]]

def tsp_exact(matrix):
    n = len(matrix)
    best_cost = float('inf')
    best_path = None

    for perm in permutations(range(1, n)):
        current_cost = sum(matrix[prev][curr] for prev, curr in zip((0,) + perm, perm + (0,)))
        if current_cost < best_cost:
            best_cost = current_cost
            best_path = (0,) + perm + (0,)

    return best_cost, best_path

def tsp_nearest(matrix):
    n = len(matrix)
    visited, path, current = [False] * n, [0], 0
    visited[current] = True

    while len(path) < n:
        next_node = min((i for i in range(n) if not visited[i]), key=lambda x: matrix[current][x])
        path.append(next_node)
        visited[next_node] = True
        current = next_node

    path.append(path[0])  # Retorna ao ponto inicial
    return calculate_path_cost(matrix, path), path

def tsp_insertion(matrix):
    n = len(matrix)
    path = [0]  
    unvisited = set(range(1, n))  

    nearest = min(unvisited, key=lambda x: matrix[path[-1]][x])
    path.append(nearest)
    unvisited.remove(nearest)

    while unvisited:
        best_cost, best_insert = float('inf'), None
        for u in unvisited:
            for i in range(len(path) - 1):
                cost = matrix[path[i]][u] + matrix[u][path[i + 1]] - matrix[path[i]][path[i + 1]]
                if cost < best_cost:
                    best_cost, best_insert = cost, (i, u)
        path.insert(best_insert[0] + 1, best_insert[1])
        unvisited.remove(best_insert[1])

    path.append(path[0])  # Retorna ao ponto inicial
    return calculate_path_cost(matrix, path), path

def solve_tsp(file_path):
    try:
        matrix = load_adjacency_matrix(file_path)
        optimal_cost = int(os.path.basename(file_path).split('_')[1].split('.')[0])
    except (FileNotFoundError, ValueError, IndexError):
        print(f"Erro ao processar o arquivo: {file_path}")
        return

    # Heurística do vizinho mais próximo
    start_time = time.time()
    approx_cost, approx_path = tsp_nearest(matrix)
    approx_time = time.time() - start_time

    # Heurística de inserção
    start_time = time.time()
    insertion_cost, insertion_path = tsp_insertion(matrix)
    insertion_time = time.time() - start_time

    # Algoritmo exato
    start_time = time.time()
    exact_cost, exact_path = tsp_exact(matrix)
    exact_time = time.time() - start_time if exact_cost else None

    print(f"Arquivo: {file_path}")
    print(f"Custo ótimo: {optimal_cost}")
    print(f"Aproximativo Vizinho Mais Próximo:\nCusto: {approx_cost}, Tempo: {approx_time:.4f}s\n")
    print(f"Aproximativo Inserção:\nCusto: {insertion_cost}, Tempo: {insertion_time:.4f}s\n")
    print(f"Exato:\nCusto: {exact_cost}, Tempo: {exact_time:.4f}s\n")

files = ["tsp1_253.txt", "tsp2_1248.txt", "tsp3_1194.txt", "tsp4_7013.txt", "tsp5_27603.txt"]
solve_tsp(files[0])
