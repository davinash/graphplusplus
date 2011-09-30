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

#ifndef _GRAPH_H__
#define _GRAPH_H__

#include <map>
#include <list>
#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <iostream>
#include <algorithm>
#include <functional>

// To avoid warning C4503:
// decorated name length exceeded, name was truncated
#pragma warning(disable:4503)


template <typename _TyV>
class Vertex {
public:
	Vertex(_TyV in) : m_Label(in),
	                  m_bVisited(false){ }
	~Vertex() { }
	bool operator < ( const Vertex & right) const {
		return m_Label < right.m_Label;
	}
	friend std::ostream& operator << (std::ostream& os, const Vertex& vertex) {
		return os << vertex.m_Label;	
	}

	_TyV getLabel() { return m_Label;}

	void SetVisited(bool bVisited) { 
		m_bVisited = bVisited ;
	}
	bool GetVisited() { return m_bVisited;}
private:
	_TyV m_Label;
	bool m_bVisited;
protected:
};

template <typename _TyI>
struct  CompareIterator {
      bool operator()(_TyI left,_TyI right) { 
		  return *left < *right; 
	  }
};

template <typename _TyG>
class Graph {
private:		
public:
	typedef typename Vertex<_TyG>							VertexType;
	typedef typename std::set<VertexType>					GraphStorage;
	typedef typename GraphStorage::iterator					GraphStorageItr;
	typedef typename GraphStorage::const_iterator			GraphStorageConstItr;

	typedef typename std::list<VertexType *>				AdjListType;
	typedef typename AdjListType::iterator					AdjListTypeItr;
	typedef typename AdjListType::const_iterator			AdjListTypeConstItr;

	typedef typename std::map<	VertexType*, 
								AdjListType, 
								CompareIterator<VertexType*> >	GraphType;

	typedef typename GraphType::iterator					GraphTypeItr;
	typedef typename GraphType::const_iterator				GraphTypeConstItr;

public:
	Graph(bool bType = true) : m_Type(bType) {}
	~Graph() {}

	void AddVertex(VertexType vertexID) {
		if ( m_GraphStorage.find(vertexID) == m_GraphStorage.end()) {
			m_GraphStorage.insert(vertexID);
		}	
	}

	AdjListType GetAdjList(VertexType vertexID) {
		VertexType *vVertexIDPtr  = &(*(m_GraphStorage.find(vertexID)));
		GraphTypeItr iter = m_Graph.find(vVertexIDPtr);
		AdjListType list;
		if ( iter != m_Graph.end()){
			return iter->second;
		}
		return list;
	}
	void AddEdge(VertexType vLeft, VertexType vRight) {
		AddVertex(vLeft);
		AddVertex(vRight);	

		VertexType *vLeftPtr  = &(*(m_GraphStorage.find(vLeft)));
		VertexType *vRightPtr = &(*(m_GraphStorage.find(vRight)));

		m_Graph[vLeftPtr].push_back(vRightPtr);
		if ( m_Type ) {
			m_Graph[vRightPtr].push_back(vLeftPtr);
		}
	}

	void AddEdge(_TyG vLevt, _TyG vRight) {
        this->AddEdge((VertexType) vLevt, (VertexType) vRight);
    }
	/* 
	 * DFS of an undirected graph is G = ( V, E ) partitions
	 * the edges in E into two sets T and B.
	 * T is called as tree edges, and B is called as back edges.
	 * The subgraph ( V, T ) is an undirected forect
	 * called a depth-first spanning forest for Graph G.
	 * 
	 * DepthFirstSearch will return depth-first spanning forest
	 *
	 */
private:
	void _init_not_visited() {
		for ( GraphStorageItr itr =  m_GraphStorage.begin();
			                  itr != m_GraphStorage.end();
							  ++itr ) {
			itr->SetVisited(false);
		}
	}

	void _DepthFirstSearch(VertexType *V) {
		std::stack<VertexType *> S;
		V->SetVisited(true);
		S.push( V );

		while ( !S.empty()) {
			VertexType *topVertex = S.top();

			AdjListType vertexAdjList = m_Graph.find(topVertex)->second;
			AdjListTypeItr itr;
			for ( itr = vertexAdjList.begin(); itr != vertexAdjList.end(); ++itr) {
				VertexType *currentVertex = (*itr);
				if (currentVertex->GetVisited() == false) {
					currentVertex->SetVisited(true);
					S.push(currentVertex);
					break;
				}
			}
			if ( itr == vertexAdjList.end()) {
				S.pop();
			}
		}
	}
public:
	void DepthFirstSearch() {
		_init_not_visited();
		for ( GraphTypeItr I = m_Graph.begin();I != m_Graph.end(); ++I) {
			if ( I->first->GetVisited() == false )
				_DepthFirstSearch(I->first);
		}
	}

	friend std::ostream& operator << (std::ostream& os, Graph& graph) {
		std::cout << "---------------------------------------------" << std::endl;
		for ( GraphStorageItr	iter = graph.m_GraphStorage.begin(); 
								iter != graph.m_GraphStorage.end(); 
								++iter) {
			std::cout << *iter << " : " ;
			AdjListType list = graph.GetAdjList(*iter);
			AdjListTypeItr iter1;
			for ( iter1 = list.begin(); iter1 != list.end(); ++iter1) {
				std::cout << *(*iter1) << '\t' ;
			}
			std::cout << std::endl;
		}
		std::cout << "---------------------------------------------" << std::endl;
		return os;
	}
private:
	bool          m_Type; 
	GraphType     m_Graph;
	GraphStorage  m_GraphStorage;
protected:
};


#endif


/*
		// Tree and Back Edges both.
		std::stack<VertexType *> TBEdges; 

		InitToFalse();

		// For all the vertex in the graph
		// There can be some non-connected comoponents
		for ( GraphStorageItr	iter = this->m_GraphStorage.begin(); 
								iter != this->m_GraphStorage.end(); 
								++iter) {
			if ( iter->isVisited() == false ) {
				TBEdges.push(&(*iter));
			}
			while (!TBEdges.empty()) {
				VertexType* vLeft = TBEdges.top();
				TBEdges.pop();
				vLeft->SetVisited(true);

				AdjListType AdjList = this->GetAdjList(*vLeft);
				for ( AdjListTypeItr	iterAdjList = AdjList.begin();  
										iterAdjList != AdjList.end();  
										++iterAdjList) {
					VertexType *vTemp = (*iterAdjList);
					if ( vTemp->isVisited() == false ) {
						TBEdges.push(&(*vTemp));
					}
				} // Adjacency loop ends here.
				VertexType *vRight = TBEdges.top();
				TBEdges.pop();
				std::cout << "Adding Edge " << *vLeft << "," << *vRight << std::endl;
				outDFST.AddEdge(*vLeft, *vRight);
				//TBEdges.pop();
			} // Tree and Back Edges loop ends here
		//} // forest loop ends here
*/