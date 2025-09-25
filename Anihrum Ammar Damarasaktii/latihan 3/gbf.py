import heapq

# Graph Romania
graph = {
    'Arad': [('Zerind', 75), ('Sibiu', 140), ('Timisoara', 118)],
    'Zerind': [('Arad', 75), ('Oradea', 71)],
    'Oradea': [('Zerind', 71), ('Sibiu', 151)],
    'Sibiu': [('Arad', 140), ('Oradea', 151), ('Fagaras', 99), ('Rimnicu Vilcea', 80)],
    'Timisoara': [('Arad', 118), ('Lugoj', 111)],
    'Lugoj': [('Timisoara', 111), ('Mehadia', 70)],
    'Mehadia': [('Lugoj', 70), ('Dobreta', 75)],
    'Dobreta': [('Mehadia', 75), ('Craiova', 120)],
    'Craiova': [('Dobreta', 120), ('Rimnicu Vilcea', 146), ('Pitesti', 138)],
    'Rimnicu Vilcea': [('Sibiu', 80), ('Craiova', 146), ('Pitesti', 97)],
    'Fagaras': [('Sibiu', 99), ('Bucharest', 178)],
    'Pitesti': [('Rimnicu Vilcea', 97), ('Craiova', 138), ('Bucharest', 101)],
    'Bucharest': [('Fagaras', 178), ('Pitesti', 101), ('Giurgiu', 90), ('Urziceni', 85)],
    'Giurgiu': [('Bucharest', 90)],
    'Urziceni': [('Bucharest', 85), ('Hirsova', 98), ('Vaslui', 142)],
    'Hirsova': [('Urziceni', 98), ('Eforie', 86)],
    'Eforie': [('Hirsova', 86)],
    'Vaslui': [('Urziceni', 142), ('Iasi', 92)],
    'Iasi': [('Vaslui', 92), ('Neamt', 87)],
    'Neamt': [('Iasi', 87)]
}

# Heuristic
heuristic = {
    'Arad': 366, 'Zerind': 374, 'Oradea': 380, 'Sibiu': 253, 'Timisoara': 329,
    'Lugoj': 244, 'Mehadia': 241, 'Dobreta': 242, 'Craiova': 160, 'Rimnicu Vilcea': 193,
    'Fagaras': 178, 'Pitesti': 98, 'Bucharest': 0, 'Giurgiu': 77, 'Urziceni': 80,
    'Hirsova': 151, 'Eforie': 161, 'Vaslui': 199, 'Iasi': 226, 'Neamt': 234
}

def path_cost(path):
    total = 0
    for i in range(len(path)-1):
        u, v = path[i], path[i+1]
        for neighbor, c in graph[u]:
            if neighbor == v:
                total += c
                break
    return total

# Greedy Best-First Search
def greedy_bfs(start, goal):
    visited = set()
    pq = [(heuristic[start], [start])]
    while pq:
        h, path = heapq.heappop(pq)
        node = path[-1]
        if node == goal:
            return path, path_cost(path)
        if node not in visited:
            visited.add(node)
            for neighbor, _ in graph[node]:
                if neighbor not in visited:
                    heapq.heappush(pq, (heuristic[neighbor], path + [neighbor]))
    return None, float('inf')

# A* Search
def a_star(start, goal):
    pq = [(heuristic[start], 0, [start])]  # (f, g, path)
    visited = {}
    while pq:
        f, g, path = heapq.heappop(pq)
        node = path[-1]
        if node == goal:
            return path, g
        if node not in visited or g < visited[node]:
            visited[node] = g
            for neighbor, cost in graph[node]:
                g_new = g + cost
                f_new = g_new + heuristic[neighbor]
                heapq.heappush(pq, (f_new, g_new, path + [neighbor]))
    return None, float('inf')


# Tes
start, goal = "Oradea", "Bucharest"
path_gbfs, cost_gbfs = greedy_bfs(start, goal)
path_astar, cost_astar = a_star(start, goal)

print("Greedy BFS Path:", path_gbfs, "Cost:", cost_gbfs)
print("A* Path:", path_astar, "Cost:", cost_astar)
