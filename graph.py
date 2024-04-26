import random
import csv

def generate_graph(num_nodes, min_weight, max_weight, filename):
    """Generate a graph in edge list format and save to a CSV file."""
    # Ensure that there are at least 2 nodes
    if num_nodes < 2:
        raise ValueError("Number of nodes must be at least 2")

    # Create a list of nodes with names "node1", "node2", ...
    nodes = [f"Node{i+1}" for i in range(num_nodes)]

    # Initialize an empty list to store edges
    edges = []

    # Generate edges between nodes
    for u in range(num_nodes - 1):
        v = random.randint(u + 1, num_nodes - 1)
        weight = random.randint(min_weight, max_weight)
        edges.append([nodes[u], nodes[v], weight])

    # Connect the last node to a randomly selected node other than the last one
    v = random.randint(0, num_nodes - 2)
    weight = random.randint(min_weight, max_weight)
    edges.append([nodes[num_nodes - 1], nodes[v], weight])

    # Write edges to file
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        for edge in edges:
            writer.writerow(edge)

# Example usage
generate_graph(num_nodes=4000, min_weight=1, max_weight=10, filename="graph.csv")
