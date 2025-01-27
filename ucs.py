import heapq
import networkx as nx
import matplotlib.pyplot as plt

def uniform_cost_search(graph, start, goal):
    priority_queue = [(0, start)]
    visited = {start: (0, None)}
    exploration_order = []

    while priority_queue:
        current_cost, current_node = heapq.heappop(priority_queue)
        exploration_order.append(current_node)

        if current_node == goal:
            return current_cost, reconstruct_path(visited, start, goal), exploration_order
        
        for neighbor, cost in graph[current_node]:
            total_cost = current_cost + cost
            if neighbor not in visited or total_cost < visited[neighbor][0]:
                visited[neighbor] = (total_cost, current_node)
                heapq.heappush(priority_queue, (total_cost, neighbor))
    
    return None, exploration_order

def reconstruct_path(visited, start, goal):
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = visited[current][1]
    path.reverse()
    return path

def visualize_graph(graph, path=None):
    G = nx.DiGraph()
    for node, edges in graph.items():
        for neighbor, cost in edges:
            G.add_edge(node, neighbor, weight=cost)

    pos = nx.spring_layout(G)
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=15, font_weight='bold', edge_color='gray')
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels, font_size=12)

    if path:
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2.5)
    
    plt.title("Uniform Cost Search Path Visualization")
    plt.show()

graph = {
    'A': [('B', 5), ('C', 1)],
    'B': [('D', 3), ('E', 2)],
    'C': [('F', 2), ('G', 3)],
    'D': [('H', 4)],
    'E': [('I', 4), ('J', 3)],
    'F': [('K', 4), ('L', 5)],
    'G': [],
    'H': [],
    'I': [],
    'J': [],
    'K': [],
    'L': []
}

start_node = 'A'
goal_node = 'D'
result = uniform_cost_search(graph, start_node, goal_node)

if result[0] is not None:
    total_cost, path = result[0], result[1]
    print(f"Least cost path from {start_node} to {goal_node}: {' -> '.join(path)} with total cost {total_cost}")
    print(f"Nodes explored in order: {''.join(result[2])}")
    visualize_graph(graph, path)
else:
    print(f"No path found from {start_node} to {goal_node}")
    print(f"Nodes explored in order: {''.join(result[1])}")
