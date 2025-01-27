import heapq
import networkx as nx
import matplotlib.pyplot as plt

def greedy_best_first_search(graph, start, goal, heuristic):
    # Priority queue to store the frontier nodes, initialized with the start node
    priority_queue = [(heuristic[start], start)]
    # Dictionary to store the visited nodes
    visited = {start: None}
    exploration_order = []

    while priority_queue:
        # Pop the node with the lowest heuristic value from the priority queue
        _, current_node = heapq.heappop(priority_queue)
        exploration_order.append(current_node)

        # If we reached the goal, return the path
        if current_node == goal:
            return reconstruct_path(visited, start, goal), exploration_order

        # Explore the neighbors
        for neighbor, cost in graph[current_node]:
            if neighbor not in visited:
                visited[neighbor] = current_node
                heapq.heappush(priority_queue, (heuristic[neighbor], neighbor))

    # If the goal is not reachable, return None
    return None, exploration_order

def reconstruct_path(visited, start, goal):
    # Reconstruct the path from start to goal by following the visited nodes
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = visited[current]
    path.reverse()
    return path

def visualize_graph(graph, path=None):
    G = nx.DiGraph()

    # Adding nodes and edges to the graph
    for node, edges in graph.items():
        for neighbor, cost in edges:
            G.add_edge(node, neighbor, weight=cost)

    pos = nx.spring_layout(G)  # Positioning the nodes

    # Drawing the graph
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=15, font_weight='bold', edge_color='gray')
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels, font_size=12)

    if path:
        # Highlight the path in red
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2.5)

    plt.title("Greedy Best-First Search Path Visualization")
    plt.show()

# Example graph represented as an adjacency list
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

# Heuristic values
heuristics = {
    'A': 7,
    'B': 3,
    'C': 6,
    'D': 2,
    'E': 5,
    'F': 4,
    'G': 2,
    'H': 0,
    'I': 0,
    'J': 0,
    'K': 0,
    'L': 0
}

# Example usage of the Greedy Best-First Search function
start_node = 'A'
goal_node = 'L'
result = greedy_best_first_search(graph, start_node, goal_node, heuristics)

if result[0] is not None:
    path = result[0]
    print(f"Path from {start_node} to {goal_node}: {' -> '.join(path)}")
    print(f"Nodes explored in order: {''.join(result[1])}")
    visualize_graph(graph, path)
else:
    print(f"No path found from {start_node} to {goal_node}")
    print(f"Nodes explored in order: {''.join(result[1])}")
