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

#include "graph.h"
#include <string>
#include <assert.h>

void BuildMyGraphShortPathTest(graph<std::string> &G) {	
	G.add_edge("v","r");
	G.add_edge("r","s");
	G.add_edge("s","w");
	G.add_edge("w","x");
	G.add_edge("w","t");
	G.add_edge("t","x");
	G.add_edge("t","u");
	G.add_edge("x","u");
	G.add_edge("x","y");
	G.add_edge("u","y");
}

void BuildMyGraphTS(graph<std::string> &G) {

	G.add_edge("A","D");
	G.add_edge("A","E");

	G.add_edge("B","E");

	G.add_edge("C","F");
	G.add_edge("C","G");
	G.add_edge("C","A");
	G.add_edge("C","B");

	G.add_edge("D","E");
	G.add_edge("D","F");

	G.add_edge("E","F");

	G.add_edge("F","H");
	
}
void BuildMyWeightedGraph(graph<std::string> &G) {
	G.add_edge("A", "B", 1);
	G.add_edge("A", "D", 2);	
	G.add_edge("A", "F", 2);

	G.add_edge("B", "D", 2);	
	G.add_edge("B", "C", 1);

	G.add_edge("C", "D", 1);
	G.add_edge("C", "E", 3);	

	G.add_vertex ("D");

	G.add_edge("E", "D", 2);
	G.add_edge("E", "G", 1);	

	G.add_edge("F", "D", 1);	

	G.add_edge("G", "D", 3);
	G.add_edge("G", "F", 3);	

}

void BuildBFGraph(graph<std::string> &G){
	G.add_edge("s","t",6);
	G.add_edge("s","y",7);
	G.add_edge("t","x",5);
	G.add_edge("t","y",8);
	G.add_edge("x","t",-2);
	G.add_edge("z","x",7);
	G.add_edge("z","s",2);
	G.add_edge("y","z",9);
	G.add_edge("y","x",-3);
	G.add_edge("t","z",-4);
}
int main(int argc, char **argv) {

	std::string expctedResult = "";
	std::string actualResult  = "";

	// Test 1
	graph<std::string> myGraphShortestPath;
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
	graph<std::string> myGraphTS(DIRECTED);
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

	// Test 3
	graph<std::string> myGraphWeight;
	BuildMyWeightedGraph(myGraphWeight);
	myGraphWeight.BuildMST();

	// Test 4
	graph<std::string> bfGraph;
	BuildBFGraph(bfGraph);
	graph<std::string>::distance_map_t p;
}
