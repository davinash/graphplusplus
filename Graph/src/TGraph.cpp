/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 *  This file is part of graphplusplus project
 *  Copyright (C) 2011 Avinash Dongre ( dongre.avinash@gmail.com )
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

#include "Graph.h"
#include <string>
#include <assert.h>

void BuildMyGraphShortPathTest(Graph<std::string> &myGraph) {	
	myGraph.AddEdge("v","r");
	myGraph.AddEdge("r","s");
	myGraph.AddEdge("s","w");
	myGraph.AddEdge("w","x");
	myGraph.AddEdge("w","t");
	myGraph.AddEdge("t","x");
	myGraph.AddEdge("t","u");
	myGraph.AddEdge("x","u");
	myGraph.AddEdge("x","y");
	myGraph.AddEdge("u","y");
}

void BuildMyGraphTS(Graph<std::string> &myGraph) {

	myGraph.AddEdge("A","D");
	myGraph.AddEdge("A","E");

	myGraph.AddEdge("B","E");

	myGraph.AddEdge("C","F");
	myGraph.AddEdge("C","G");
	myGraph.AddEdge("C","A");
	myGraph.AddEdge("C","B");

	myGraph.AddEdge("D","E");
	myGraph.AddEdge("D","F");

	myGraph.AddEdge("E","F");

	myGraph.AddEdge("F","H");
	
}
void BuildMyWeightedGraph(Graph<std::string> &myGraph) {
	myGraph.AddEdge("A", "B", 1);
	myGraph.AddEdge("A", "D", 2);	
	myGraph.AddEdge("A", "F", 2);

	myGraph.AddEdge("B", "D", 2);	
	myGraph.AddEdge("B", "C", 1);

	myGraph.AddEdge("C", "D", 1);
	myGraph.AddEdge("C", "E", 3);	

	myGraph.AddVertex ("D");

	myGraph.AddEdge("E", "D", 2);
	myGraph.AddEdge("E", "G", 1);	

	myGraph.AddEdge("F", "D", 1);	

	myGraph.AddEdge("G", "D", 3);
	myGraph.AddEdge("G", "F", 3);	

}
int main(int argc, char **argv) {

	std::string expctedResult = "";
	std::string actualResult  = "";

	// Test 1
	Graph<std::string> myGraphShortestPath;
	BuildMyGraphShortPathTest(myGraphShortestPath);
	std::vector<std::string> path;
	myGraphShortestPath.GetShortestPath("r","y",path);
	expctedResult = "rswxy";
	actualResult  = "";
	for ( std::vector<std::string>::iterator itr  = path.begin();
		itr != path.end(); ++itr) {
			actualResult.append(*itr);
	}
	assert ( expctedResult == actualResult);
	expctedResult = "";
	actualResult  = "";

	// Test 2
	Graph<std::string> myGraphTS(DIRECTED);
	BuildMyGraphTS(myGraphTS);
	std::list<std::string> listTS;
	myGraphTS.SortToplogical(listTS);
	expctedResult = "CGBADEFH";
	for ( std::list<std::string>::iterator itr  = listTS.begin();
		itr != listTS.end(); ++itr) {
			actualResult.append(*itr);
	}
	assert ( expctedResult == actualResult);
	expctedResult = "";
	actualResult  = "";

	// Test 2
	Graph<std::string> myGraphWeight;
	BuildMyWeightedGraph(myGraphWeight);
	myGraphWeight.BuildMST();
}
