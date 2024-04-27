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

// Define the Node struct
struct Node {
    int vertex;
    int dist;
    Node* child;
    Node* nextSibling;
    Node* prev;
    Node(int v, int d) : vertex(v), dist(d), child(nullptr), nextSibling(nullptr), prev(nullptr) {}
};

class PairingHeap {
private:
    Node* root;

    Node* mergePairs(Node* firstSibling);
    Node* merge(Node* a, Node* b);

public:
    PairingHeap() : root(nullptr) {}
    ~PairingHeap();

    bool empty() const;
    void insert(int vertex, int dist);
    int findMin() const;
    void deleteMin();
    void decreaseKey(int vertex, int newDist);
};

PairingHeap::~PairingHeap() {
    // Release memory
    // This can be done using a post-order traversal and deleting each node
}

bool PairingHeap::empty() const {
    return root == nullptr;
}

void PairingHeap::insert(int vertex, int dist) {
    Node* newNode = new Node(vertex, dist);
    if (root == nullptr) {
        root = newNode;
    } else {
        root = merge(root, newNode);
    }
}

int PairingHeap::findMin() const {
    if (root == nullptr)
        throw std::logic_error("Heap is empty");
    return root->vertex;
}

void PairingHeap::deleteMin() {
    if (root == nullptr)
        return;

    Node* oldRoot = root;
    root = mergePairs(root->child);
    delete oldRoot;
}

Node* PairingHeap::mergePairs(Node* firstSibling) {
    if (firstSibling == nullptr || firstSibling->nextSibling == nullptr)
        return firstSibling;

    std::vector<Node*> pairs;
    Node* curr = firstSibling;
    while (curr != nullptr) {
        Node* next = curr->nextSibling;
        curr->nextSibling = nullptr;
        Node* sibling = next != nullptr ? next->nextSibling : nullptr;
        pairs.push_back(merge(curr, next));
        curr = sibling;
    }

    Node* newRoot = nullptr;
    for (int i = 0; i < pairs.size(); i += 2) {
        if (i == pairs.size() - 1) {
            if (newRoot == nullptr)
                newRoot = pairs[i];
            else
                newRoot = merge(newRoot, pairs[i]);
        } else {
            newRoot = merge(newRoot, merge(pairs[i], pairs[i + 1]));
        }
    }

    return newRoot;
}

Node* PairingHeap::merge(Node* a, Node* b) {
    if (a == nullptr) return b;
    if (b == nullptr) return a;

    if (a->dist > b->dist) {
        b->nextSibling = a->child;
        if (a->child != nullptr)
            a->child->prev = b;
        a->child = b;
        b->prev = a;
        return a;
    } else {
        a->nextSibling = b->child;
        if (b->child != nullptr)
            b->child->prev = a;
        b->child = a;
        a->prev = b;
        return b;
    }
}

void PairingHeap::decreaseKey(int vertex, int newDist) {
    // Decrease the key of the vertex to the new distance
    // This operation can be implemented as a cut and link operation in the tree
    // In a pairing heap, you would need to find the node corresponding to the vertex,
    // update its distance, and potentially adjust the tree structure to maintain the heap property
}

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
    PairingHeap pq;

    dist[src] = 0;
    pq.insert(src, 0);

    while (!pq.empty()) {
        int u = pq.findMin();
        pq.deleteMin();

        sptSet[u] = true;

        for (const auto& edge : graph[u]) {
            int v = edge.first;
            int weight = edge.second;
            if (!sptSet[v] && dist[u] != std::numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.insert(v, dist[v]);
            }
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
