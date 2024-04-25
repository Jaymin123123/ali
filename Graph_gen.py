import random
import csv

def generate_fully_connected_graph(num_nodes, min_weight, max_weight, filename):
    """Generate a fully connected graph in edge list format and save to a CSV file."""
    edges = []
    for u in range(num_nodes):
        for v in range(u + 1, num_nodes):
            weight = random.randint(min_weight, max_weight)
            edges.append([f"Node{u}", f"Node{v}", weight])

    # Write edges to file
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        for edge in edges:
            writer.writerow(edge)

# Example usage
generate_fully_connected_graph(num_nodes=6000, min_weight=1, max_weight=30, filename="fully_connected_graphw1000.csv")