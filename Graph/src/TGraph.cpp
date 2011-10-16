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


int main(int argc, char **argv) {
	graph<std::string> g;
	g.insert("A","B");
  g.breadh_first_search();
}




































	//BuildMyGraphShortPathTest(myGraphShortestPath);
	//std::vector<std::string> path;
	//myGraphShortestPath.GetShortestPath("r","y",path);
	//expctedResult = "rswxy";
	//actualResult  = "";
	//for ( std::vector<std::string>::iterator itr  = path.begin();
	//	itr != path.end(); ++itr) {
	//		actualResult.append(*itr);
	//}
	//assert ( expctedResult == actualResult);
	//expctedResult = "";
	//actualResult  = "";

	//// Test 2
	//graph<std::string> myGraphTS(DIRECTED);
	//BuildMyGraphTS(myGraphTS);
	//std::list<std::string> listTS;
	//myGraphTS.SortToplogical(listTS);
	//expctedResult = "CGBADEFH";
	//for ( std::list<std::string>::iterator itr  = listTS.begin();
	//	itr != listTS.end(); ++itr) {
	//		actualResult.append(*itr);
	//}
	//assert ( expctedResult == actualResult);
	//expctedResult = "";
	//actualResult  = "";

	//// Test 3
	//graph<std::string> myGraphWeight;
	//BuildMyWeightedGraph(myGraphWeight);
	//myGraphWeight.BuildMST();

	//// Test 4
	//graph<std::string> bfGraph;
	//BuildBFGraph(bfGraph);
	//graph<std::string>::distance_map_t p;
//}



//void BuildMyGraphShortPathTest(graph<std::string> &G) {	
//	G.insert("v","r");
//	G.insert("r","s");
//	G.insert("s","w");
//	G.insert("w","x");
//	G.insert("w","t");
//	G.insert("t","x");
//	G.insert("t","u");
//	G.insert("x","u");
//	G.insert("x","y");
//	G.insert("u","y");
//}
//
//void BuildMyGraphTS(graph<std::string> &G) {
//
//	G.insert("A","D");
//	G.insert("A","E");
//
//	G.insert("B","E");
//
//	G.insert("C","F");
//	G.insert("C","G");
//	G.insert("C","A");
//	G.insert("C","B");
//
//	G.insert("D","E");
//	G.insert("D","F");
//
//	G.insert("E","F");
//
//	G.insert("F","H");
//	
//}
//void BuildMyWeightedGraph(graph<std::string> &G) {
//	G.insert("A", "B", 1);
//	G.insert("A", "D", 2);	
//	G.insert("A", "F", 2);
//
//	G.insert("B", "D", 2);	
//	G.insert("B", "C", 1);
//
//	G.insert("C", "D", 1);
//	G.insert("C", "E", 3);	
//
//	G.add_vertex ("D");
//
//	G.insert("E", "D", 2);
//	G.insert("E", "G", 1);	
//
//	G.insert("F", "D", 1);	
//
//	G.insert("G", "D", 3);
//	G.insert("G", "F", 3);	
//
//}
//
//void BuildBFGraph(graph<std::string> &G){
//	G.insert("s","t",6);
//	G.insert("s","y",7);
//	G.insert("t","x",5);
//	G.insert("t","y",8);
//	G.insert("x","t",-2);
//	G.insert("z","x",7);
//	G.insert("z","s",2);
//	G.insert("y","z",9);
//	G.insert("y","x",-3);
//	G.insert("t","z",-4);
//}