import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import random
import heapq

# Set random seed for reproducibility
seed = 0
random.seed(seed)
np.random.seed(seed)

# Create the graph and add nodes
Gn = nx.Graph()
nodes = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J1", "J2", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U"]

# Define positions for visualization
pos = {
    "A": (10, 10), "B": (9.5, 10), "C": (9.2, 9),
    "D": (6, 6), "E": (6.5, 10), "F": (6, 10),
    "G": (4.5, 10.2), "H": (4.5, 7), "I": (3.5, 9.7),
    "J1": (2.5, 9), "J2": (2, 6), "K": (1.5, 9.5),
    "L": (3, 8), "M": (2, 15), "N": (3, 15),
    "O": (6.5, 15), "P": (7.5, 14), "Q": (8, 14),
    "R": (9, 14), "S": (7, 17), "T": (10.7, 15),
    "U": (0.2, 8)
}

# Function to calculate Euclidean distance between two points
def euclidean_distance(pos1, pos2):
    return np.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

# List of edges with connections between nodes
edges = [("A","B"),
         ("B","T"),("B","R"),("B","C"),
         ("C","D"),
         ("D","E"),("D","H"),
         ("E","F"),("E","O"),
         ("F","G"),("F","H"),
         ("G","H"),("G","I"),
         ("H","L"),
         ("I","J1"),("I","N"),
         ("J1","K"),("J1","L"),
         ("J2","L"),("J2","U"),
         ("K","U"),
         ("M","N"),
         ("N","O"),
         ("O","P"),
         ("P","S"),("P","Q"),
         ("Q","R"),
         ("R","T")]

# Add edges to the graph with Euclidean distance as weight
for edge in edges:
    node1, node2 = edge
    distance = euclidean_distance(pos[node1], pos[node2])
    Gn.add_edge(node1, node2, weight=distance)

# Uniform Cost Search (UCS) algorithm implementation
def uniform_cost_search(graph, start, goal):
    # Priority queue to store nodes based on the current total path cost
    # Priority queue stores (cumulative cost, current node, path taken so far)
    priority_queue = [(0, start, [start])]
    visited = set()  # To keep track of visited nodes

    while priority_queue:
        # Get the node with the lowest cumulative cost
        cumulative_cost, current_node, path = heapq.heappop(priority_queue)

        # If we reached the goal, return the path and the total cost
        if current_node == goal:
            return path, cumulative_cost

        # Skip if the current node has been visited
        if current_node in visited:
            continue
        visited.add(current_node)

        # Explore neighbors
        for neighbor in graph.neighbors(current_node):
            if neighbor not in visited:
                # Get the edge weight (Euclidean distance)
                edge_weight = graph[current_node][neighbor]['weight']
                new_cost = cumulative_cost + edge_weight
                new_path = path + [neighbor]
                # Push the neighbor into the priority queue with updated cost and path
                heapq.heappush(priority_queue, (new_cost, neighbor, new_path))

    # Return None if the goal is not reachable
    #return None, float('inf')

# Function to display UCS result and return the path for visualization
def display_ucs_result(start, goal):
    print(f"UCS path from {start} to {goal}")
    path, total_cost = uniform_cost_search(Gn, start, goal)
    if path:
        print(f"Path found: {' -> '.join(path)}")
        print(f"Total cost: {total_cost:.2f}")
        return path
    else:
        print(f"No path found from {start} to {goal}.")
        return None

# Visualize the graph with node positions and optionally highlight a path
def visualize_graph(graph, path=None):
    plt.figure(figsize=(10, 10))
    
    # Draw nodes and edges
    nx.draw(graph, pos, with_labels=True, node_color="green", font_size=10, font_color = "white")
    
    # Get edge labels (weights) to display on the graph
    edge_labels = nx.get_edge_attributes(graph, 'weight')
    
    # Round the edge weights to two decimal places for readability
    edge_labels = {key: f'{value:.2f}' for key, value in edge_labels.items()}
    
    # Draw edge labels
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_color='green')

    # Highlight the path found by UCS, if any
    if path:
        path_edges = [(path[i], path[i+1]) for i in range(len(path) - 1)]
        nx.draw_networkx_edges(graph, pos, edgelist=path_edges, width=3, edge_color='r')
    
    plt.show()

# Get user input for the start and goal nodes
def get_user_input():
    print("Available nodes:", ", ".join(nodes))
    start_node = input("Enter the start node: ").strip().upper()
    goal_node = input("Enter the goal node: ").strip().upper()
    
    # Validate user input
    if start_node not in nodes or goal_node not in nodes:
        print("Invalid input! Please enter valid nodes.")
        return get_user_input()  # Retry input
    return start_node, goal_node

# Get user input
algo = int(input ("UCS [1] or A* [2] ? "))
if algo == 1:
    while True:
        start_node, goal_node = get_user_input()
        if start_node in nodes:
            path = display_ucs_result(start_node, goal_node)
            if path:  # Only visualize if a valid path is found
                visualize_graph(Gn, path)
            break
        print("Invalid starting node. Please choose from the available nodes.")
elif algo == 2:
    print("yay")
#start_node, goal_node = get_user_input()

# Run UCS and visualize the result
#path = display_ucs_result(start_node, goal_node)

# Visualize the graph and the path found by UCS
#if path:  # Only visualize if a valid path is found
 #   visualize_graph(Gn, path)
