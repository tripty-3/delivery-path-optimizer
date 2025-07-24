/*
 * Delivery Path Optimizer
 * 
 * This program helps optimize delivery routes between locations by:
 * - Managing a network of locations and routes
 * - Calculating optimal paths using Dijkstra's algorithm
 * - Simulating delivery routes using BFS
 * 
 * Key Data Structures:
 * - Graph represented using adjacency lists
 * - Priority queue for Dijkstra's algorithm
 * - Queue for BFS simulation
 */

// Standard Template Library (STL) headers
#include <iostream>       // For input/output operations
#include <vector>         // Dynamic array implementation
#include <queue>          // For priority_queue and queue
#include <unordered_map>  // Hash table implementation for fast lookups
#include <limits>         // For numeric_limits (infinity representation)
#include <algorithm>      // For remove_if algorithm

using namespace std; // Standard namespace to avoid std:: prefixes

/*
 * DeliveryPathOptimizer class
 * Manages locations, routes, and path optimization algorithms
 */
class DeliveryPathOptimizer {
private:
    // Maps location names to their indices for quick lookup
    unordered_map<string, int> locationToIndex;
    
    // Maps indices back to location names for display
    vector<string> indexToLocation;
    
    // Adjacency list representation of the graph
    // Each entry is a vector of pairs: (neighbor_index, cost)
    vector<vector<pair<int, int>>> adjList;
    
    // Total number of locations in the system
    int locationCount;

public:
    // Constructor - initializes with 0 locations
    DeliveryPathOptimizer() : locationCount(0) {}

    /*
     * Adds a new location to the delivery network
     * @param name: Name of the location to add
     */
    void addLocation(const string& name) {
        // Check if location already exists
        if (locationToIndex.count(name)) {
            cout << "Location already exists.\n";
            return;
        }
        
        // Add to both mappings
        locationToIndex[name] = locationCount++; // Assign and increment index
        indexToLocation.push_back(name); // Add to name list
        adjList.emplace_back(); // Add empty adjacency list for new location
        
        cout << "Location '" << name << "' added.\n";
    }

    /*
     * Removes a location from the delivery network
     * @param name: Name of the location to remove
     */
    void removeLocation(const string& name) {
        // Check if location exists
        if (!locationToIndex.count(name)) {
            cout << "Location not found.\n";
            return;
        }

        // Get the index of the location to remove
        int idx = locationToIndex[name];
        
        // Remove from adjacency list
        adjList.erase(adjList.begin() + idx);
        
        // Remove from name list
        indexToLocation.erase(indexToLocation.begin() + idx);

        // Remove all routes to/from this location and adjust indices
        for (auto& neighbors : adjList) {
            // Remove any routes to the deleted location
            neighbors.erase(remove_if(neighbors.begin(), neighbors.end(),
                [idx](const pair<int, int>& p) { return p.first == idx; }), neighbors.end());

            // Adjust indices of locations after the removed one
            for (auto& p : neighbors) {
                if (p.first > idx) p.first--;
            }
        }

        // Rebuild the locationToIndex mapping
        locationToIndex.clear();
        for (int i = 0; i < indexToLocation.size(); ++i) {
            locationToIndex[indexToLocation[i]] = i;
        }
        
        locationCount--; // Decrement total location count
        cout << "Location '" << name << "' removed.\n";
    }

    /*
     * Adds a bidirectional route between two locations
     * @param from: Starting location
     * @param to: Destination location
     * @param cost: Time or distance cost between locations
     */
    void addRoute(const string& from, const string& to, int cost) {
        // Check if both locations exist
        if (!locationToIndex.count(from) || !locationToIndex.count(to)) {
            cout << "One or both locations not found.\n";
            return;
        }
        
        // Get indices for both locations
        int u = locationToIndex[from], v = locationToIndex[to];
        
        // Add to both adjacency lists (undirected graph)
        adjList[u].push_back(make_pair(v, cost));
        adjList[v].push_back(make_pair(u, cost));
        
        cout << "Route from '" << from << "' to '" << to << "' added with cost " << cost << ".\n";
    }

    /*
     * Removes a route between two locations
     * @param from: Starting location
     * @param to: Destination location
     */
    void removeRoute(const string& from, const string& to) {
        // Check if both locations exist
        if (!locationToIndex.count(from) || !locationToIndex.count(to)) {
            cout << "One or both locations not found.\n";
            return;
        }
        
        // Get indices for both locations
        int u = locationToIndex[from], v = locationToIndex[to];

        // Remove from first location's adjacency list
        auto& listU = adjList[u];
        listU.erase(remove_if(listU.begin(), listU.end(),
            [v](const pair<int, int>& p) { return p.first == v; }), listU.end());

        // Remove from second location's adjacency list
        auto& listV = adjList[v];
        listV.erase(remove_if(listV.begin(), listV.end(),
            [u](const pair<int, int>& p) { return p.first == u; }), listV.end());

        cout << "Route between '" << from << "' and '" << to << "' removed.\n";
    }

    /*
     * Displays all locations in the system
     */
    void showLocations() const {
        cout << "\nLocations:\n";
        for (const auto& name : indexToLocation) {
            cout << "- " << name << "\n";
        }
    }

    /*
     * Calculates and displays optimal delivery paths from a starting location
     * using Dijkstra's algorithm
     * @param start: Starting location for path calculation
     */
    void optimizeDeliveryPlan(const string& start) const {
        // Check if starting location exists
        if (!locationToIndex.count(start)) {
            cout << "Starting location not found.\n";
            return;
        }

        // Initialize distance vector with "infinity"
        int n = locationCount;
        vector<int> dist(n, numeric_limits<int>::max());
        
        // Distance to start is 0
        int src = locationToIndex.at(start);
        dist[src] = 0;

        // Priority queue for Dijkstra's algorithm (min-heap)
        // Stores pairs of (distance, vertex_index)
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push(make_pair(0, src));

        // Main Dijkstra's algorithm loop
        while (!pq.empty()) {
            pair<int, int> top = pq.top(); pq.pop();
            int d = top.first;  // Current distance
            int u = top.second; // Current vertex

            // Skip if we've already found a better path
            if (d > dist[u]) continue;

            // Explore all neighbors
            for (size_t i = 0; i < adjList[u].size(); ++i) {
                int v = adjList[u][i].first;       // Neighbor index
                int cost = adjList[u][i].second;    // Edge cost
                
                // Relaxation step
                if (dist[v] > dist[u] + cost) {
                    dist[v] = dist[u] + cost;       // Update distance
                    pq.push(make_pair(dist[v], v)); // Add to queue
                }
            }
        }

        // Display results
        cout << "\n--- Optimized Delivery Plan from '" << start << "' ---\n";
        for (int i = 0; i < n; ++i) {
            cout << indexToLocation[i] << ": ";
            if (dist[i] == numeric_limits<int>::max())
                cout << "Unreachable\n";
            else
                // Assuming cost is 5 times the time (example conversion)
                cout << "ETA = " << dist[i] << ", Cost = " << dist[i] * 5 << "\n";
        }
    }

    /*
     * Simulates delivery route using Breadth-First Search (BFS)
     * @param start: Starting location for simulation
     */
    void simulateDelivery(const string& start) const {
        // Check if starting location exists
        if (!locationToIndex.count(start)) {
            cout << "Starting location not found.\n";
            return;
        }

        // BFS initialization
        queue<int> q;                   // Queue for BFS
        vector<bool> visited(locationCount, false); // Visited markers
        int src = locationToIndex.at(start); // Starting index

        q.push(src);
        visited[src] = true;

        cout << "\n--- Route Simulation ---\n";
        while (!q.empty()) {
            int curr = q.front(); q.pop();
            cout << "Delivering to: " << indexToLocation[curr] << "\n";

            // Visit all neighbors
            for (size_t i = 0; i < adjList[curr].size(); ++i) {
                int neighbor = adjList[curr][i].first;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
    }
};

/*
 * Main Driver Menu
 * Provides interactive interface for using the DeliveryPathOptimizer
 */
int main() {
    DeliveryPathOptimizer dpo; // Create optimizer instance
    string input;              // For user input
    int choice;                // Menu choice
    string loc1, loc2;         // Location names
    int cost;                  // Route cost

    // Main menu loop
    while (true) {
        cout << "\n=== Delivery Path Optimizer Menu ===\n";
        cout << "1. Add Location\n2. Remove Location\n3. Add Route\n4. Remove Route\n";
        cout << "5. Show Locations\n6. Optimize Delivery Plan\n7. Simulate Route\n8. Exit\n";
        cout << "Enter choice: ";
        getline(cin, input);

        // Convert input to integer with error handling
        try {
            choice = stoi(input);
        } catch (...) {
            cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        // Process menu choice
        switch (choice) {
            case 1: // Add Location
                cout << "Enter location name: ";
                getline(cin, loc1);
                dpo.addLocation(loc1);
                break;
                
            case 2: // Remove Location
                cout << "Enter location name to remove: ";
                getline(cin, loc1);
                dpo.removeLocation(loc1);
                break;
                
            case 3: // Add Route
                cout << "Enter FROM location: ";
                getline(cin, loc1);
                cout << "Enter TO location: ";
                getline(cin, loc2);
                cout << "Enter cost/time: ";
                getline(cin, input);
                try {
                    cost = stoi(input);
                    dpo.addRoute(loc1, loc2, cost);
                } catch (...) {
                    cout << "Invalid cost input.\n";
                }
                break;
                
            case 4: // Remove Route
                cout << "Enter FROM location: ";
                getline(cin, loc1);
                cout << "Enter TO location: ";
                getline(cin, loc2);
                dpo.removeRoute(loc1, loc2);
                break;
                
            case 5: // Show Locations
                dpo.showLocations();
                break;
                
            case 6: // Optimize Delivery Plan
                cout << "Enter starting location: ";
                getline(cin, loc1);
                dpo.optimizeDeliveryPlan(loc1);
                break;
                
            case 7: // Simulate Route
                cout << "Enter starting location for simulation: ";
                getline(cin, loc1);
                dpo.simulateDelivery(loc1);
                break;
                
            case 8: // Exit
                cout << "Exiting...\n";
                return 0;
                
            default:
                cout << "Invalid choice. Try again.\n";
        }
    }
    return 0;
}
