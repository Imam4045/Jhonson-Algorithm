#include<bits/stdc++.h>
using namespace std;

/* Function to detect negative weight cycles using Bellman-Ford
 * @Parameters:
 *  - const vector<vector<pair<int, int>>> &graph: Adjacency list of the graph.
 *  - vector<int> &h: Vector to store shortest distances from a virtual source.
 * Functionality: Computes shortest paths and checks for negative weight cycles.
 * Returns true if no negative cycle is found; false otherwise.
 */
bool bellmanFord(const vector<vector<pair<int, int>>> &graph, vector<int> &h) {
    
    int n = graph.size();
    h = vector<int>(n, INT_MAX);
    h[n - 1] = 0; 

    for (int i = 0; i < n; i++) { 
        for (int u = 0; u < n; u++) {
            if (h[u] == INT_MAX) continue;
            for (auto &edge : graph[u]) {
                int v = edge.first, weight = edge.second;
                if (h[v] > h[u] + weight) {
                    h[v] = h[u] + weight;
                    if (i == n - 1) {
                        return false; // Negative weight cycle found
                    }
                }
            }
        }
    }
    return true;
}

/* Function to find shortest paths using Dijkstra's algorithm
 * @Parameters:
 *  - const vector<vector<pair<int, int>>> &graph: Adjacency list of the graph.
 *  - int src: Source node to calculate shortest paths from.
 * Functionality: Computes shortest distances from the source to all nodes using a priority queue.
 * Returns a vector of distances from the source node.
 */
vector<int> dijkstra(const vector<vector<pair<int, int>>> &graph, int src) {
    int n = graph.size();
    vector<bool> visited(n, false);  
    vector<int> dist(n, INT_MAX); // O(v)
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    dist[src] = 0;
    pq.push({0, src});
            // distance , node name
    // bfs
    while (!pq.empty()) { 
        int u = pq.top().second; // node itself
        pq.pop(); 

        if (visited[u]) continue; // Skip already visited nodes
        visited[u] = true;
    
        for (auto &edge : graph[u]) {  
            int v = edge.first, weight = edge.second;  // first is V is 2nd is weght
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

/* Function to compute all-pairs shortest paths using Johnson's algorithm
 * @Parameters:
 *  - const vector<vector<pair<int, int>>> &graph: Adjacency list of the graph.
 * Functionality: Detects negative weight cycles using Bellman-Ford, re-weights edges, 
 *                and calculates shortest paths using Dijkstra for each vertex.
 * Outputs shortest path distances or indicates if no path exists.
 */
void johnson(const vector<vector<pair<int, int>>> &graph) {
    int V = graph.size() - 1; // Number of vertices

    vector<int> h; // Holds the h values from Bellman-Ford

    if (!bellmanFord(graph, h)) {
        cout << "Negative weight cycle detected\n";
        return;
    }

    // Re-weight all the edges
    vector<vector<pair<int, int>>> newGraph(graph);
    for (int u = 0; u < V; u++) {
        for (auto &edge : newGraph[u]) {
            edge.second += h[u] - h[edge.first];
        }
    }

    // Run Dijkstra from each vertex
    for (int u = 0; u < V; u++) {
        vector<int> dist = dijkstra(newGraph, u);
        // Adjust the distances to get the original distances
        for (int v = 0; v < V; v++) {
            if (dist[v] < INT_MAX) {
                dist[v] -= h[u] - h[v];
                cout << "Shortest distance from " << u << " to " << v << " is " << dist[v] << "\n";
            } else {
                cout << "No path from " << u << " to " << v << "\n";
            }
        }
    }
}

int main() {
    int V, E; // Number of vertices
    cout << "Vertices: ";
    cin >> V;
    cout << "Edges: ";
    cin >> E;

    // Graph represented as adjacency list
    vector<vector<pair<int, int>>> graph(V + 1); // Adding extra vertex for Bellman-Ford

    // Add edges
    for(int i = 0; i < E; i++){
        int u, v, weight;
        cin >> u >> v >> weight;
        graph[u].push_back({v, weight});
    }

    // Source node connecting to all others (for Johnson's Algorithm)
    for(int i = 0; i < V; i++){
        graph[V].push_back({i,0});
    }


    johnson(graph);
    return 0;
}



