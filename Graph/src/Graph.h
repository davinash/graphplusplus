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
#include <deque>

// To avoid warning C4503:
// decorated name length exceeded, name was truncated
#pragma warning(disable:4503)

enum VertexColor  {
	BLACK,
	WHITE,
	GRAY
};

enum GraphType {
	DIRECTED,
	UNDIRECTED
};

template <typename _TyV>
class Vertex {
public:
	Vertex(_TyV in) :	m_Label(in){ }
	~Vertex() { }
	bool operator < ( const Vertex & right) const {
		return m_Label < right.m_Label;
	}

	bool operator == ( const Vertex & right ) const {
		return m_Label == right.m_Label;
	}

	friend std::ostream& operator << (std::ostream& os, const Vertex& vertex) {
		return os << vertex.m_Label;	
	}

	_TyV getLabel() { return m_Label;}
private:
	_TyV m_Label;
public:
	VertexColor m_Color;
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
	typedef typename Vertex<_TyG>						Vertex_t;
	typedef typename std::set<Vertex_t>					GraphStorage;
	typedef typename GraphStorage::iterator				GraphStorageItr;
	typedef typename GraphStorage::const_iterator		GraphStorageConstItr;

	typedef typename std::list<Vertex_t *>				AdjList_t;
	typedef typename AdjList_t::iterator				AdjListItr_t;
	typedef typename AdjList_t::const_iterator			AdjListTypeConstItr;

	typedef typename std::map<	Vertex_t*, 
						AdjList_t, 
						CompareIterator<Vertex_t*> >	Graph_t;

	typedef typename Graph_t::iterator					GraphTypeItr;
	typedef typename Graph_t::const_iterator			GraphTypeConstItr;

public:
	Graph(GraphType eType = UNDIRECTED) : m_Type(eType) {
		m_BFSdone = false;
	}
	~Graph() {}

	void AddVertex(Vertex_t vertexID) {
		if ( m_GraphStorage.find(vertexID) == m_GraphStorage.end()) {
			m_GraphStorage.insert(vertexID);
		}	
		m_BFSdone = false; // any modification done
	}

	AdjList_t GetAdjList(Vertex_t vertexID) {
		Vertex_t *vVertexIDPtr  = &(*(m_GraphStorage.find(vertexID)));
		GraphTypeItr iter = m_Graph.find(vVertexIDPtr);
		AdjList_t list;
		if ( iter != m_Graph.end()){
			return iter->second;
		}
		return list;
	}
	void AddEdge(Vertex_t vLeft, Vertex_t vRight) {
		AddVertex(vLeft);
		AddVertex(vRight);	

		Vertex_t *vLeftPtr  = &(*(m_GraphStorage.find(vLeft)));
		Vertex_t *vRightPtr = &(*(m_GraphStorage.find(vRight)));

		m_Graph[vLeftPtr].push_back(vRightPtr);
		if ( m_Type == UNDIRECTED) {
			m_Graph[vRightPtr].push_back(vLeftPtr);
		}
		m_BFSdone = false;
	}

	void AddEdge(_TyG vLevt, _TyG vRight) {
        this->AddEdge((Vertex_t) vLevt, (Vertex_t) vRight);
    }

	void BFS(Vertex_t *pStart = (Vertex_t*)0) {
		if ( ! pStart) {
			pStart = m_Graph.begin()->first;
		}
		_BFS(pStart);
		m_BFSdone = true;
	}

	void print_path(_TyG S, _TyG V) {

		Vertex_t *pS  = (Vertex_t*)0;
		Vertex_t *pV =  (Vertex_t*)0;

		if ( m_GraphStorage.find(S) != m_GraphStorage.end()){
			pS  = &(*(m_GraphStorage.find(S)));
		}
		if ( m_GraphStorage.find(V) != m_GraphStorage.end()){
			pV  = &(*(m_GraphStorage.find(V)));
		}

		if ( !pS || !pV){
			return;
		}

		if ( ! m_BFSdone ) {			
			_BFS(pS);
		}
		std::vector<Vertex_t *> pathVector;
		_print_path(pS, pV, pathVector);

		std::vector<Vertex_t *>::iterator itr ;
		for (itr =  pathVector.begin();itr != pathVector.end() - 1; ++itr) {
			std::cout << *(*itr) << " --> " ;
		}
		std::cout << *(*itr);
		std::cout << std::endl;
	}
	
private:
	void _print_path(Vertex_t *S,Vertex_t *V, std::vector<Vertex_t *> & pathVector){
		if (*S == *V) {
			pathVector.push_back(V);
		} else {
			if ( m_bfs_parent[V] == ( Vertex_t*)0)
				std::cout << "No Parth from " << *S << " to" << *V << "exists" << std::endl;
			else
				_print_path(S,m_bfs_parent[V],pathVector);

			pathVector.push_back(V);
		}
	}
	void _BFS(Vertex_t *S) {
		// 1. Paint every vertex as WHITE
		// 2. Set the d to -1
		// 3. Set the p of each vertex to NULL
		for ( GraphTypeItr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
			Vertex_t *U = itr->first;
			U->m_Color =  WHITE;
			m_bfs_Distance[U] = -1;
			m_bfs_parent[U] = (Vertex_t *)0;
		}
		S->m_Color = GRAY;
		// Initialize the d to 0
		m_bfs_Distance[S] = 0;
		m_bfs_parent[S] = (Vertex_t *)0;

		std::deque<Vertex_t*> Q;
		// Enque s ( given vertex ) to Queue Q
		Q.push_back(S);
		while ( !Q.empty()) {
			Vertex_t *U  = Q.front();
			Q.pop_front();
			AdjList_t vAdjList = m_Graph.find(U)->second;

			for ( AdjListItr_t itr = vAdjList.begin(); itr != vAdjList.end(); ++itr) {
				Vertex_t *V = *(itr);
				if ( V->m_Color == WHITE ) {
					V->m_Color = GRAY;
					m_bfs_Distance[V] = m_bfs_Distance[U] + 1;
					m_bfs_parent[V] = U;
					Q.push_back(V);
				}
			}
			U->m_Color = BLACK;
		} // Queue empty loop
	}
public:
	friend std::ostream& operator << (std::ostream& os, Graph& graph) {
		std::cout << "---------------------------------------------" << std::endl;
		for ( GraphStorageItr	iter = graph.m_GraphStorage.begin(); 
								iter != graph.m_GraphStorage.end(); 
								++iter) {
			std::cout << *iter << " : " ;
			AdjList_t list = graph.GetAdjList(*iter);
			AdjListItr_t iter1;
			for ( iter1 = list.begin(); iter1 != list.end(); ++iter1) {
				std::cout << *(*iter1) << '\t' ;
			}
			std::cout << std::endl;
		}
		std::cout << "---------------------------------------------" << std::endl;
		return os;
	}
private:
	GraphType m_Type; 
	Graph_t   m_Graph;
	GraphStorage  m_GraphStorage;

	// Distance map
	std::map<	Vertex_t*, 
				int, 
				CompareIterator<Vertex_t*> > m_bfs_Distance;
	// Parent Map
	std::map<	Vertex_t*, 
				Vertex_t*, 
				CompareIterator<Vertex_t*> > m_bfs_parent;
	//is BFS algorithm executed
	bool m_BFSdone;

protected:
};


#endif


/*
		// Tree and Back Edges both.
		std::stack<Vertex_t *> TBEdges; 

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
				Vertex_t* vLeft = TBEdges.top();
				TBEdges.pop();
				vLeft->SetVisited(true);

				AdjList_t AdjList = this->GetAdjList(*vLeft);
				for ( AdjListItr_t	iterAdjList = AdjList.begin();  
										iterAdjList != AdjList.end();  
										++iterAdjList) {
					Vertex_t *vTemp = (*iterAdjList);
					if ( vTemp->isVisited() == false ) {
						TBEdges.push(&(*vTemp));
					}
				} // Adjacency loop ends here.
				Vertex_t *vRight = TBEdges.top();
				TBEdges.pop();
				std::cout << "Adding Edge " << *vLeft << "," << *vRight << std::endl;
				outDFST.AddEdge(*vLeft, *vRight);
				//TBEdges.pop();
			} // Tree and Back Edges loop ends here
		//} // forest loop ends here
*/