/*
 * Name: Caroline
 * Date Submitted: 03/28/2019
 * Lab Section: 003
 * Assignment Name: Lab 9 - Graph Algorithms
 */

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "graphalgs-solution.h"

using namespace std;

node fill_graph(graphalgs& graph, ifstream& infile);

int main() {
	ifstream infile("graphInput.txt");
	int testNumber = 1;
	int switcher = -1;

	do {
		graphalgs graph;
		node sourceNode = fill_graph(graph, infile);

		// Determine which algorithm to run
		cout << "Select an option: \n (1) Dijkstra \n (2) Kruskal" << endl;
		cin >> switcher;

		cout << "---Test " << testNumber++ << "---" << endl;
		if (switcher == 1) {
			graph.dijkstra(sourceNode);
		} else if (switcher == 2) {
			graph.kruskal();
		} else {
			cout << "Invalid option, terminating test" << endl;
		}
	} while (infile.is_open());

	return 0;
}

//helper function which determines if a given char is whitespace
bool is_white_space(char value) {
	return value == '\t' || value == '\n' || value == '\r' || value == '\f' || value == ' ';
}

//gets rid of any trailing and/or leading white space when lines from file are read in
string trim_string(string value) {
	auto start = value.begin();
	auto end = value.rbegin();
	int length = value.length();
	while (is_white_space(*start)) start++;
	while (is_white_space(*end)) { end++; length--; }
	return string(start, start + length);
}

//helper function which reads a file and creates a graph from the input
node fill_graph(graphalgs& graph, ifstream& infile) {
	node sourceNode = "";
	string line;

	bool continueReading = true;

	while (continueReading && getline(infile, line)) {
		line = trim_string(line);
		stringstream stream(line);
		if (line == "#####") {
			continueReading = false;
			continue;
		}

		node node1, node2;
		int distance;

		getline(stream, node1, ',');
		getline(stream, node2, ',');
		stream >> distance;

		if (sourceNode == "")
			sourceNode = node1;
		graph.add_edge(node1, node2, distance);
	}

	if (continueReading)
		infile.close();
	return sourceNode;
}
