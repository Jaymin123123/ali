#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <utility>
#include <chrono>

// CUDA kernel function to compute shortest distances from the source vertex
__global__ void dijkstraKernel(int* graph, int* dist, int* visited, int V, int src) {
    int u = threadIdx.x;

    if (u == src) {
        dist[u] = 0;
    } else {
        dist[u] = INT_MAX;
    }
    
    visited[u] = 0;

    __syncthreads(); // Synchronize threads before proceeding

    // Main Dijkstra's algorithm loop
    for (int i = 0; i < V - 1; ++i) {
        // Find the vertex with the minimum distance
        int minDist = INT_MAX;
        int minIndex = -1;

        for (int j = 0; j < V; ++j) {
            if (!visited[j] && dist[j] < minDist) {
                minDist = dist[j];
                minIndex = j;
            }
        }

        // Mark the selected vertex as visited
        int u = minIndex;
        visited[u] = 1;

        // Update distances for neighboring vertices
        for (int v = 0; v < V; ++v) {
            int weight = graph[u * V + v];
            if (!visited[v] && weight > 0) {
                int newDist = dist[u] + weight;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                }
            }
        }
    }
}

// Helper function to read graph from file
std::vector<int> readGraph(const std::string& filename, int& V) {
    std::ifstream file(filename);
    std::vector<int> graph;

    std::string line;
    std::getline(file, line); // Read the first line to get the number of vertices
    V = std::stoi(line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        int weight;
        while (ss >> weight) {
            graph.push_back(weight);
        }
    }

    return graph;
}

int main() {
    std::string filename = "fully_connected_graphw1000.txt";
    int V;
    std::vector<int> graph = readGraph(filename, V);
    
    // Allocate memory for graph, distances, and visited arrays on the GPU
    int* d_graph;
    int* d_dist;
    int* d_visited;
    cudaMalloc(&d_graph, V * V * sizeof(int));
    cudaMalloc(&d_dist, V * sizeof(int));
    cudaMalloc(&d_visited, V * sizeof(int));
    
    // Copy graph data from CPU to GPU memory
    cudaMemcpy(d_graph, graph.data(), V * V * sizeof(int), cudaMemcpyHostToDevice);
    
    // Set up block and grid dimensions for CUDA kernel invocation
    int threadsPerBlock = V;
    int blocksPerGrid = 1;
    
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    
    // Invoke CUDA kernel function
    dijkstraKernel<<<blocksPerGrid, threadsPerBlock>>>(d_graph, d_dist, d_visited, V, 0);
    
    // Wait for all CUDA threads to finish
    cudaDeviceSynchronize();
    
    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;  
    std::cout << "Execution time: " << elapsed.count() << " seconds.\n";
    
    // Copy distances array from GPU to CPU memory
    int* h_dist = new int[V];
    cudaMemcpy(h_dist, d_dist, V * sizeof(int), cudaMemcpyDeviceToHost);
    
    // Output shortest distances
    std::cout << "Vertex\tDistance from Source\n";
    for (int i = 0; i < V; ++i) {
        std::cout << i << "\t" << h_dist[i] << "\n";
    }
    
    // Free GPU memory
    cudaFree(d_graph);
    cudaFree(d_dist);
    cudaFree(d_visited);
    
    // Free CPU memory
    delete[] h_dist;

    return 0;
}
