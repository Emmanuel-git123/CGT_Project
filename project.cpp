#include<bits/stdc++.h>
using namespace std;

enum NodeType { AMBULANCE, HOUSE, HOSPITAL, INTERSECTION ,DISASTER};
 
struct Node {
    int id;
    NodeType type;
    string name;
    int drain;
}; 
 
struct Edge {
    int to;
    int weight;
};

// BFS
void bfs(int start, const vector<vector<Edge>> &adj, const vector<Node> &nodes, vector<bool> &visited) {
    queue<int> q;
    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (auto &e : adj[u]) {
            int v = e.to;
            if (!visited[v] && nodes[v].type != DISASTER) {
                visited[v] = true;
                q.push(v);
            }
        }
    }
}

void checkHouseConnectivity(const vector<vector<Edge>> &adj, const vector<Node> &nodes) {
    int n = nodes.size();
    vector<bool> reachable(n, false);

    for (auto &node : nodes) {
        if (node.type == HOSPITAL) {
            vector<bool> visited(n, false);
            bfs(node.id, adj, nodes, visited);
            for (int i = 0; i < n; i++) {
                if (visited[i]) reachable[i] = true;
            }
        }
    }
    cout << endl << "BFS Algorithm" << endl;
    cout << "Houses not reachable from any hospital: ";
    bool allConnected = true;
    for (auto &node : nodes) {
        if (node.type == HOUSE && !reachable[node.id]) {
            cout << node.name << " (ID " << node.id << ") is disconnected from Hospital\n";
            allConnected = false;
        }
    }

    if (allConnected) {
        cout << "None" << endl;
        cout << "All houses are reachable from hospitals.\n";
    }
    cout << endl;
}


class GridGraph {
public:
    int n;
    vector<Node> nodes;
    vector<vector<Edge>> adj;
 
    GridGraph(int size) : n(size) {
        int total = n * n;
        adj.resize(total);
        nodes.resize(total);
        for (int i = 0; i < total; i++) {
            nodes[i].id = i;
            nodes[i].type = INTERSECTION;
            nodes[i].name = "Intersection_" + to_string(i);
        }
    }
 
    int nodeID(int row, int col) {
        return row * n + col;
    }

    void addEdge(int u, int v, int weight) {
        adj[u].push_back({ v, weight });
        adj[v].push_back({ u, weight });
    }
 
    void buildGrid() {
        mt19937 gen(time(0));
        uniform_int_distribution<> dis(1, 10);
        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                int current = nodeID(r, c);
                if (r > 0) {
                    int w = dis(gen);
                    addEdge(current, nodeID(r - 1, c),dis(gen));
                    cout << "Edge added: " << nodes[current].name << " <-> " << nodes[nodeID(r-1, c)].name << " (Weight " << w << ")" << endl;
                }
                if (c > 0) {
                    int w = dis(gen);
                    addEdge(current, nodeID(r, c - 1),dis(gen));
                    cout << "Edge added: " << nodes[current].name << " <-> " << nodes[nodeID(r, c-1)].name << " (Weight " << w << ")" << endl;
                }
            }
        }
        uniform_int_distribution<> drain(1,10);
        for(auto &node : nodes) node.drain=drain(gen);
    }
 
    void assignNodeType(int id, NodeType type, string name = "") {
        nodes[id].type = type;
        if (!name.empty()) nodes[id].name = name;
    }

    void simulateRain(){
        mt19937 gen(time(0));
        uniform_int_distribution<> ran(1, 4);
        int severity = ran(gen);
 
        for(auto &node : nodes) {
            if(node.drain <= severity) node.type = DISASTER;
        }
        damage();
    }
 
    void damage(){
        cout << "Damaged nodes:" << endl;
        for (auto& node : nodes) {
            if (node.type == DISASTER) {
                cout << node.name << " (ID " << node.id << ")" << endl;
            }
        }
        cout << endl;
    }

    vector<int> getActiveDegrees() {
        vector<int> degree(nodes.size(), 0);

        for (int u = 0; u < (int)nodes.size(); u++) {
            if (nodes[u].type == DISASTER) continue;

            for (auto &edge : adj[u]) {
                int v = edge.to;
                if (nodes[v].type != DISASTER) {
                    degree[u]++;
                }
            }
        }
        cout << "\nActive node degrees (excluding damaged nodes):\n";
        for (int i = 0; i < (int)nodes.size(); i++) {
            if (nodes[i].type != DISASTER)
                cout << nodes[i].name << " (ID " << i << "): " << degree[i] << endl;
        }

        return degree;
    }

    //dijkstra algorithm
    vector<int> dijkstra(int src, int dest, int& total_cost) {
        int V = n * n;
        vector<int> dist(V, numeric_limits<int>::max());
        vector<int> vis(V);
        vector<int> prev(V, -1);
        dist[src] = 0;
 
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        vector<int> path;
 
        if(nodes[src].type == DISASTER || src == -1 || dest == -1) return path;
        pq.push({ 0, src });
 
        while (!pq.empty()) {
            auto d= pq.top().first;
            auto u= pq.top().second;
            pq.pop();
 
            if (vis[u]==1)
                continue;
            vis[u]=1;
 
            for (auto& edge : adj[u]) {
                int v = edge.to;
                int weight = edge.weight;
                if(nodes[v].type == DISASTER)
                    continue;
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({ dist[v], v });
                }
            }
        }
 
        total_cost = dist[dest];
 
        if (total_cost == numeric_limits<int>::max())
            return path;
 
        for (int at = dest; at != -1; at = prev[at]) path.push_back(at);
        reverse(path.begin(), path.end());
        return path;
    }
 
    int findNearest(int from, NodeType type) {
        int best = -1;
        int minDist = numeric_limits<int>::max();
        int dummyCost;
        for (auto& node : nodes) {
            if (node.type == type && node.type != DISASTER) {
                vector<int> path = dijkstra(from, node.id, dummyCost);
                if (!path.empty() && dummyCost < minDist) {
                    minDist = dummyCost;
                    best = node.id;
                }
            }
        }
        return best;
    }
 
    void emergency(int from) {
        int costToHouse;
        int nearestAmbulance = findNearest(from, AMBULANCE);
 
        if(nearestAmbulance == -1) {
            cout << "There are no Ambulances available at the moment" << endl;
            return;
        }
        else{
            cout << "Finding shortest path from ambulance to emergency house..." << endl;
            vector<int> pathToHouse = dijkstra(nearestAmbulance, from, costToHouse);
            cout << "Path: ";
            printPath(pathToHouse);
            cout << "Distance: " << costToHouse << endl;
        }
        
        cout << "Finding nearest hospital from the emergency house..." << endl;
        int nearestHospital = findNearest(from, HOSPITAL);
        if(nearestHospital == -1) {
            cout << "No hospital found in the map." << endl;
            return;
        }
        else{
            int costToHospital;
            vector<int> pathToHospital = dijkstra(from, nearestHospital, costToHospital);
            cout << "Path: ";
            printPath(pathToHospital);
            cout << "Distance: " << costToHospital << endl;
        }
        cout << endl;
    }
 
    void printPath(const vector<int>& path) {
        for (size_t i = 0; i < path.size(); i++) {
            cout << nodes[path[i]].name;
            if (i != path.size() - 1) cout << " -> ";
        }
        cout << endl;
    }

    //minimun spanning tree
    void minimumSpanningTree() {
        int V = n*n;
        vector<bool> inMST(V, false);
        vector<int> key(V, INT_MAX);
        vector<int> parent(V, -1);

        // Use Prim's algorithm
        int start = -1;
        for(auto &node : nodes){
            if(node.type == HOUSE){
                start = node.id;
                break;
            }
        }
        if(start == -1){
            cout << "No houses to connect in MST.\n";
            return;
        }

        key[start] = 0;

        for(int count = 0; count < V-1; count++){
            int u = -1;
            int minKey = INT_MAX;
            for(int v=0; v<V; v++){
                if(!inMST[v] && key[v] < minKey && nodes[v].type != DISASTER){
                    minKey = key[v];
                    u = v;
                }
            }

            if(u == -1) break;
            inMST[u] = true;

            for(auto &e : adj[u]){
                int v = e.to;
                int w = e.weight;
                if(!inMST[v] && nodes[v].type != DISASTER && w < key[v]){
                    key[v] = w;
                    parent[v] = u;
                }
            }
        }
        cout << "\nMinimum Spanning Tree for houses (or active nodes):\n";
        for(int i=0; i<V; i++){
            if(parent[i] != -1){
                cout << nodes[parent[i]].name << " -- " << nodes[i].name << " (Weight " << key[i] << ")\n";
            }
        }
    }

};

int main() {
    int n = 5;
    GridGraph g(n);
    g.buildGrid();

    g.assignNodeType(0, AMBULANCE, "AmbulanceStation");
    g.assignNodeType(12, HOUSE, "House-12");
    g.assignNodeType(24, HOSPITAL, "CityHospital");
    g.assignNodeType(6, HOUSE, "House-6");
    g.assignNodeType(18, HOSPITAL, "CountryHospital");
    g.assignNodeType(15, HOUSE, "House-15");

    cout << endl;
    g.simulateRain();

    vector<vector<Edge>> adj=g.adj;
    vector<Node> nodes=g.nodes;
    checkHouseConnectivity(adj, nodes);

    cout << "Dijkstra algorithm" << endl;
    int emergencyHouse = 15;
    g.emergency(emergencyHouse);

    cout << endl << "MST Algorithm";
    g.minimumSpanningTree();

    return 0;
}