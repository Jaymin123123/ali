#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <utility>
#include <map>
#include <chrono>
#include <set>

typedef std::pair<int, int> Edge; // Edge(destination, weight)
typedef std::vector<std::vector<Edge>> Graph; // Graph as an adjacency list

Graph readGraph(const std::string& filename, std::map<std::string, int>& nodeMap) {
    std::ifstream file(filename);
    Graph graph;
    std::string line;
    int nodeIndex = 0;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string node1, node2, weight;

        if (!getline(ss, node1, ',') || !getline(ss, node2, ',') || !getline(ss, weight)) {
            std::cerr << "Warning: Malformed line skipped: " << line << std::endl;
            continue;
        }

        if (nodeMap.find(node1) == nodeMap.end()) nodeMap[node1] = nodeIndex++;
        if (nodeMap.find(node2) == nodeMap.end()) nodeMap[node2] = nodeIndex++;
        int n1 = nodeMap[node1];
        int n2 = nodeMap[node2];
        int w = std::stoi(weight);

        if (graph.size() <= std::max(n1, n2)) {
            graph.resize(std::max(n1, n2) + 1);
        }

        graph[n1].push_back({n2, w});
        // If the graph is undirected, uncomment the next line
        // graph[n2].push_back({n1, w});
    }
    return graph;
}

void dijkstra(const Graph& graph, int src) {
    int V = graph.size();
    std::vector<int> dist(V, std::numeric_limits<int>::max());
    std::vector<bool> sptSet(V, false); // Shortest path tree set

    dist[src] = 0;

    for (int count = 0; count < V - 1; count++) {
        // Find the minimum distance vertex from the set of vertices not yet processed
        int u = -1;
        for (int v = 0; v < V; v++)
            if (!sptSet[v] && (u == -1 || dist[v] < dist[u]))
                u = v;

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (const auto& edge : graph[u]) {
            int v = edge.first;
            int weight = edge.second;
            if (!sptSet[v] && dist[u] != std::numeric_limits<int>::max() && dist[u] + weight < dist[v])
                dist[v] = dist[u] + weight;
        }
    }
}

int main() {
    std::string filename = "fully_connected_graphw1000.csv"; // Ensure this path is correct
    std::map<std::string, int> nodeMap;
    Graph graph = readGraph(filename, nodeMap);
    int source = 0; // Change this to your source vertex

    auto start = std::chrono::high_resolution_clock::now(); // Start timing
    dijkstra(graph, source);
    auto end = std::chrono::high_resolution_clock::now(); // End timing

    std::chrono::duration<double> elapsed = end - start; // Calculate elapsed time
    std::cout << "Execution time: " << elapsed.count() << " seconds.\n";

    return 0;
}
