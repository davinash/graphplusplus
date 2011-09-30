#include "Graph.h"
#include <string>

void BuildMyGraph( Graph<std::string> &myGraph) {

	myGraph.AddEdge("v1","v5");
	myGraph.AddEdge("v5","v6");
	myGraph.AddEdge("v6","v1");
	myGraph.AddEdge("v1","v3");
	myGraph.AddEdge("v1","v2");
	myGraph.AddEdge("v3","v2");
	myGraph.AddEdge("v2","v4");
	myGraph.AddEdge("v1","v4");
	myGraph.AddEdge("v10", "v11");
}

int main(int argc, char **argv) {

	Graph<std::string> myGraph;
	BuildMyGraph(myGraph);

	std::cout << myGraph;

	myGraph.DepthFirstSearch();

}
