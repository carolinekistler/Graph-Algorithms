/*
 * Name: Caroline
 * Date Submitted: 03/28/2019
 * Lab Section: 003
 * Assignment Name: Lab 9 - Graph Algorithms
 */

#pragma once

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <climits>
#include <cmath>
using namespace std;

typedef string node;

typedef struct {
    node node1;
    int distance;
    bool duplicate;
} edge;

typedef struct {
    node node1;
    node node2;
    int distance;
    bool duplicate;
} connectedNodes;


class graphalgs {
 private:
    map<node, vector<edge>> graph;
    map<node, node> nodesAndParents;
 public:
    void add_edge(node node1, node node2, int distance);
    void dijkstra(node firstNode);
    void kruskal();
    node findRoot(node);
    void unify(node node1, node node2);
};


// used to compares total mins for dijkstra
typedef pair<node, int> findMinimum;
struct compareMin {
    bool operator()(const findMinimum& left, const findMinimum& right) const {
        return left.second < right.second;
	}
};


// adds two nodes that are directly connected and their distances
// all edges will be added before dijkstra and kruskal are called
void graphalgs::add_edge(node node1, node node2, int distance) {
    edge edges;
    vector<edge> vectorOfEdges;
    // set node and distance attached to main node
    edges.node1 = node2;
    edges.distance = distance;
    edges.duplicate = false;
    // add edge to vector
    vectorOfEdges.push_back(edges);

    if (graph.count(node1)) {
        // if map contains key add value
        graph[node1].push_back(edges);
    } else {
        // if map does not contain key add key and value pair
        graph.emplace(node1, vectorOfEdges);
    }

    // create edge in other direction  and does same steps as above
    edge edges2;
    vector<edge> vectorOfEdges2;
    edges2.node1 = node1;
    edges2.distance = distance;
    edges2.duplicate = true;
    vectorOfEdges2.push_back(edges2);
    if (graph.count(node2)) {
       graph[node2].push_back(edges2);
    } else {
        graph.emplace(node2, vectorOfEdges2);
    }
}

// Calculates the minimum distance from the <sourceNode>
// to all other nodes and prints
void graphalgs::dijkstra(node sourceNode) {
    // create two maps for nodes that are visited and unvisited
    // that contains the node and distance from starting node
    map<node, int> unvisited;
    map<node, int> visited;

    // iterates through graph
    for (auto it : graph) {
        // iterate through edge
        for (auto it2 : it.second) {
            // gets nodes of edge
            node node1 = it.first;
            node node2 = it2.node1;
            auto find = unvisited.find(node1);
            auto find2 = unvisited.find(node2);
            // if the node is not visited put it in map with distance of infinity
            if (find == unvisited.end()) {
                unvisited.emplace(node1, INT_MAX);
            }
            if (find2 == unvisited.end()) {
                unvisited.emplace(node2, INT_MAX);
            }
        }
    }
    // set distance of source node to 0
    unvisited[sourceNode] = 0;
    // only run while unvisited is not empty
    while (unvisited.size() > 0) {
        // generate minimum neighbor
        auto minimum = min_element(begin(unvisited), end(unvisited), compareMin());
        // will made a vector of edges of each minimum node and edge
        vector<edge> nodesAndEdge = graph.find(minimum->first)->second;

        for (auto iter : nodesAndEdge) {
            node neighbor = iter.node1;
            // will only check distance if neighbor is unvisited
            if (visited.find(neighbor) == visited.end()) {
                auto iter2 = unvisited.find(neighbor);
                int newDistance = iter.distance + minimum->second;
                int oldDistance = iter2->second;
                // ensures the smallest distance is always saved
                if (oldDistance > newDistance) {
                    iter2->second = newDistance;
                }
            }
        }
        // will place the minimun node just visited in visited map
        visited.emplace(minimum->first, minimum->second);
        // deleted from unvisited map
        unvisited.erase(minimum);
    }
	
    // formatted output 
    cout << "Dijkstra (from " << sourceNode << ")" << endl;
    for (auto it : visited) {
        if (it.first != sourceNode) {
            cout << it.first << ": " << it.second << endl;
        }
    }
    cout << endl;
}


// compares the distance of two nodes away from each other
bool compareDistance(connectedNodes node1, connectedNodes node2) {
    return (node1.distance < node2.distance); 
}

//finds the parent of the node inserted
node graphalgs::findRoot(node mainNode) {
    node parent = nodesAndParents[mainNode];
    //only if parent is found it will return
    if (parent == mainNode) {
       return parent;
    } else {
       // uses recursion to find the root if parent is not equal to main node
       parent = findRoot(parent);
    }

    return parent;
}

//takes roots of set and changes them to all have the same root
void graphalgs::unify(node node1, node node2) {
    node1 = findRoot(node1);
    node2 = findRoot(node2);
    // unifies the two roots to have same root
	if (node1 != node2) {
        nodesAndParents[node1] = nodesAndParents[node2];
    }
}

//Generates a minimum spanning tree and prints
void graphalgs::kruskal() {
    // contain two nodes and distance between them
	connectedNodes nestedVector;
    // contains the whole graph
    vector<connectedNodes> wholeGraph;
    int totalWeight = 0;
    // iters through map and get key
	for (auto it1 : graph) {
        // iters through edges for key
        for (auto it2 : it1.second) {
            // does not iterate through same edge twice
			if (!it2.duplicate) {
                // puts key and values into struct
                nestedVector.node1 = it1.first;
                nestedVector.node2 = it2.node1;
                nestedVector.distance = it2.distance;
                // puts two nodes and distance into vector
                wholeGraph.push_back(nestedVector);
            }
        }
    }
    // sorts vector from smallest to largest values
    sort(wholeGraph.begin(), wholeGraph.end(), compareDistance);
    // initalizes nodes and parents map for each to have root of themselves
	for (int it = 0; it < wholeGraph.size(); it++) {
        nodesAndParents.emplace(wholeGraph.at(it).node1, wholeGraph.at(it).node1);
        nodesAndParents.emplace(wholeGraph.at(it).node2, wholeGraph.at(it).node2);
    }

    cout << "Kruskal" << endl;
    int count = 0;
    // iterates through vector
    for (auto it : wholeGraph) {
        // gets root node of both nodes of edge
		nodesAndParents[it.node1] = findRoot(it.node1);
        nodesAndParents[it.node2] = findRoot(it.node2);
        // if nodes are not apart of the same set
        if (nodesAndParents[it.node1] != nodesAndParents[it.node2]) {
            // make node apare of the same set and add weight
			unify(nodesAndParents[it.node1], nodesAndParents[it.node2]);
            totalWeight += it.distance;
            count++;
            cout << it.node1 << ", " << it.node2 << ", " << it.distance << endl;
        }
		// ensures that it does not iterate past the map size
        if (count >= graph.size()) {
            break;
        }
		
    }
    cout << "total weight = " << totalWeight << endl << endl;
}
