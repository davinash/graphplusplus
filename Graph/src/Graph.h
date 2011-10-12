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
#include <list>
#include <climits>

// To avoid warning C4503:
// decorated name length exceeded, name was truncated
#pragma warning(disable:4503)

enum vertex_color  {
	BLACK,
	WHITE,
	GREY
};

enum graph_type {
	DIRECTED,
	UNDIRECTED
};

enum graph_operation {
	TOPOLOGICALSORT
};

template <typename U>
class union_find {
private:
	struct element {
		U m_value;
		int m_parent;
		int m_rank;

		element(const U& value, int parent)
			:   m_value(value), m_parent(parent), m_rank(0)
		{}
	};
private:
	std::map<int,element> m_Elements;
public:
	union_find(){}
public:
	void AddElement(int x, const U& value = U()) {
		m_Elements.insert(std::make_pair(x, element(value, x)));
	}
	int FindSet(int x) {
		element& element = GetElement(x);
		int& parent = element.m_parent;
		if(parent != x) {
			parent = FindSet(parent);
		}
		return parent;
	}
	void UnionSets(int x, int y) {
		int left  = FindSet(x);
		int right = FindSet(y);
		if(left != right) SetupParentRelationship(left, right);
	}
private:
	element& GetElement(int x) {
		typename std::map<int,element>::iterator it = m_Elements.find(x);
		return it->second;
	}

	void SetupParentRelationship(int x, int y) {
		element& elementX = GetElement(x);
		element& elementY = GetElement(y);
		int& rankX = elementX.m_rank;
		int& rankY = elementY.m_rank;
		if(rankX > rankY) {
			elementY.m_parent = x;
		} else {
			elementX.m_parent = y;
			if(rankX == rankY) ++rankY;
		}
	}
};

template <typename V>
class vertex {
public:
	vertex(V in) :	m_data(in){ }
	~vertex() { }
	vertex( vertex const &right) : m_data(right.m_data){}

	bool operator < ( const vertex & right) const {
		return m_data < right.m_data;
	}

	bool operator == ( const vertex & right ) const {
		return m_data == right.m_data;
	}

	friend std::ostream& operator << (std::ostream& os, 
		const vertex& vertex) {
			return os << vertex.m_data;	
	}
	V get_remote_data() { return m_data;}
private:
	V m_data;
protected:
};

template <typename I>
struct  comp_iterator {
	bool operator()(I left,I right) { 
		return *left < *right; 
	}
};

template <typename P>
struct  comp_pair {
	bool operator()(P left,P right) { 
		return (*(left.first) < *(right.first) && 
			*(left.second) < *(right.second)); 
	}
};

template <typename G>
class graph {
private:		
public:
	typedef typename vertex<G>                              vertex_t;
	typedef typename std::set<vertex_t>                     graph_storage_t;
	typedef typename std::list<vertex_t *>                  adj_list_t;    
	typedef typename std::map<vertex_t*, adj_list_t, 
		                      comp_iterator<vertex_t*> >    graph_t;    
	typedef typename std::map<vertex_t*, int, 
		                      comp_iterator<vertex_t*> >    distance_map_t;
	typedef typename distance_map_t TimeMap_t;
	typedef typename std::map<vertex_t*, vertex_t*, 
		                      comp_iterator<vertex_t*> >    parent_map_t;
	typedef typename std::pair<vertex_t*, vertex_t*>        edge_t ;
	typedef typename std::vector<std::pair<int, edge_t> >   edge_vector_t;	
	typedef typename std::map<edge_t, int,
		                      comp_pair<edge_t> >           edge_weight_t;
	typedef typename std::map<vertex_t*, vertex_color>      color_map_t;
	typedef typename graph_storage_t::iterator              graph_storage_itr;
	typedef typename adj_list_t::iterator                   adj_list_itr;
	typedef typename graph_t::iterator                      graph_itr;
	typedef typename edge_vector_t::iterator                edge_vector_iter;

public:
	graph(graph_type eType = UNDIRECTED) : m_Type(eType) {
		m_TopologicalSortList = std::list<vertex_t *>();
	}

	~graph() {}

	void add_vertex(vertex_t vertexID) {
		graph_storage_itr pos = m_GraphStorage.find(vertexID);
		if ( pos == m_GraphStorage.end()) {
			m_GraphStorage.insert(vertexID);
			m_Graph[&(*(m_GraphStorage.find(vertexID)))] = adj_list_t();
		}	
	}

	void add_vertex(G vVertex) {
		this->add_vertex((vertex_t) vVertex);
	}

	adj_list_t GetAdjList(vertex_t vertexID) {
		vertex_t *vVertexIDPtr  = &(*(m_GraphStorage.find(vertexID)));
		graph_itr iter = m_Graph.find(vVertexIDPtr);
		adj_list_t list;
		if ( iter != m_Graph.end()){
			return iter->second;
		}
		return list;
	}

	void add_edge(vertex_t vLeft, vertex_t vRight) {
		add_vertex(vLeft);
		add_vertex(vRight);	

		vertex_t *vLeftPtr  = &(*(m_GraphStorage.find(vLeft)));
		vertex_t *vRightPtr = &(*(m_GraphStorage.find(vRight)));

		m_Graph[vLeftPtr].push_back(vRightPtr);
		if ( m_Type == UNDIRECTED) {
			m_Graph[vRightPtr].push_back(vLeftPtr);
		}
	}

	void add_edge(G vLeft, G vRight) {
		this->add_edge((vertex_t) vLeft, (vertex_t) vRight);
	}

	void add_edge(G vLeft, G vRight, int weight) {
		this->add_edge((vertex_t) vLeft, (vertex_t) vRight);

		vertex_t *vLeftPtr  = &(*(m_GraphStorage.find(vLeft)));
		vertex_t *vRightPtr = &(*(m_GraphStorage.find(vRight)));

		m_EdgeVector.push_back(std::make_pair(weight, 
			std::make_pair(vLeftPtr,
			vRightPtr)));

		m_EdgeWeight.insert(std::make_pair(
			std::make_pair(vLeftPtr,vRightPtr),
			weight));
	}

	void BFS(vertex_t *pStart = (vertex_t*)0) {
		if ( ! pStart) {
			pStart = m_Graph.begin()->first;
		}
		_BFS(pStart);

	}

	void DFS(vertex_t *pStart = (vertex_t*)0) {
		if ( ! pStart) {
			pStart = m_Graph.begin()->first;
		}
		_DFS(pStart,0);
	}


	bool GetShortestPath(G S, G V, std::vector<G> &outPath) {
		vertex_t *pS  = (vertex_t*)0;
		vertex_t *pV =  (vertex_t*)0;
		if ( m_GraphStorage.find(S) != m_GraphStorage.end()){
			pS  = &(*(m_GraphStorage.find(S)));
		}
		if ( m_GraphStorage.find(V) != m_GraphStorage.end()){
			pV  = &(*(m_GraphStorage.find(V)));
		}
		if ( !pS || !pV){
			return false;
		}
		_BFS(pS);
		_GetShortestPath(pS, pV, outPath);
		return true;
	}

	bool SortToplogical( std::list<G> &outList) {
		m_Operation = TOPOLOGICALSORT;
		_DFS(m_Graph.begin()->first);
		for ( std::list<vertex_t *>::iterator it = m_TopologicalSortList.begin();
			it != m_TopologicalSortList.end(); ++it) {
				outList.push_back((*it)->get_remote_data());
		}
		m_TopologicalSortList.clear();
		return true;
	}

	void BuildMST() {
		union_find<vertex_t*> ufSet;
		int index = 0;
		std::map<vertex_t*, int> VertexArray;
		//MAKE-SET(v)
		for ( graph_itr it = m_Graph.begin();it != m_Graph.end(); ++it) {
			vertex_t *U = it->first;
			ufSet.AddElement(index, U);
			VertexArray.insert(std::make_pair(U,index));
			index++;
		}
		//sort the edges of E into nondecreasing order by weight w
		std::sort(m_EdgeVector.begin(), m_EdgeVector.end());
		for ( edge_vector_iter it = m_EdgeVector.begin(); 
			                                  it != m_EdgeVector.end(); ++it) {
			vertex_t *pLeft  = (it->second).first;
			vertex_t *pRight = (it->second).second;
			int leftIndex    = (VertexArray.find(pLeft))->second;
			int rightIndex   = (VertexArray.find(pRight))->second;
			if ( ufSet.FindSet(leftIndex) != ufSet.FindSet(rightIndex)){
				ufSet.UnionSets(leftIndex,rightIndex);
			}
		}
	}    
private:
	void _relax(vertex_t* u, vertex_t* v, int w, 
		        distance_map_t &d, parent_map_t &p){

		int dv = d.find(v)->second;
		int du = d.find(u)->second;

		if(dv > (du + w)) {
			d[v] = du + w;
			p[v] = u;
		}
	}

	bool _GetShortestPath(vertex_t *S, vertex_t *V, 
		                               std::vector<G> & pathVector){
		if (*S == *V) {
			pathVector.push_back(V->get_remote_data());
		} else {
			if ( m_BFSParentMap[V] == ( vertex_t*)0)
				return false;
			else
				_GetShortestPath(S,m_BFSParentMap[V],pathVector);

			pathVector.push_back(V->get_remote_data());
		}
		return true;
	}

	void _BFS(vertex_t *S) {
		color_map_t colorMap;
		m_BFSDistanceMap.clear();
		m_BFSParentMap.clear();
		for ( graph_itr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
			vertex_t *U = itr->first;
			//U->m_Color =  WHITE;
			colorMap[U] = WHITE;
			m_BFSDistanceMap[U] = -1;
			m_BFSParentMap[U] = (vertex_t *)0;
		}
		//S->m_Color = GREY;
		colorMap[S] = GREY;
		// Initialize the d to 0
		m_BFSDistanceMap[S] = 0;
		m_BFSParentMap[S] = (vertex_t *)0;

		std::deque<vertex_t*> Q;
		// Enque s ( given vertex ) to Queue Q
		Q.push_back(S);
		while ( !Q.empty()) {
			vertex_t *U  = Q.front();
			Q.pop_front();
			adj_list_t vAdjList = m_Graph.find(U)->second;

			for ( adj_list_itr itr = vAdjList.begin(); 
				                            itr != vAdjList.end(); ++itr) {
				vertex_t *V = *(itr);
				if ( colorMap.find(V)->second == WHITE ) {
					colorMap[V] = GREY;
					m_BFSDistanceMap[V] = m_BFSDistanceMap[U] + 1;
					m_BFSParentMap[V] = U;
					Q.push_back(V);
				}
			}
			colorMap[U] = BLACK;
		} // Queue empty loop
	}

	void _DFS(vertex_t *S) {
		color_map_t colorMap;
		m_DFSStartTimeMap.clear();
		m_DFSFinishTimeMap.clear();
		m_DFSParentMap.clear();		
		for ( graph_itr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
			colorMap[itr->first] = WHITE;
			m_DFSParentMap[itr->first] = (vertex_t*)0;
		}
		int time = 0;
		for ( graph_itr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
			if ( colorMap.find(itr->first)->second == WHITE ) {
				_DFS_Visit(	itr->first,
					m_DFSStartTimeMap,
					m_DFSFinishTimeMap,
					m_DFSParentMap, 
					time,
					colorMap);
			}
		}
	}

	void _DFS_Visit(vertex_t *inU, TimeMap_t & d, 
		TimeMap_t & f, parent_map_t & p , 
		int & time, color_map_t &colorMap) {
			std::stack<vertex_t *> S;
			S.push(inU);
			while ( !S.empty()) {
				vertex_t *U  = S.top();				
				adj_list_t vAdjList = m_Graph.find(U)->second;
				adj_list_itr itr;
				if ( colorMap.find(U)->second == WHITE ) {  
					time = time + 1;
					d[U] = time;
					colorMap[U] = GREY;
					for ( itr = vAdjList.begin(); itr != vAdjList.end(); ++itr){
						vertex_t *V = *(itr);
						if ( colorMap.find(V)->second == WHITE ) {
							p[V] = U;
							S.push(V);
							break;
						}
					}
				} else {
					itr = vAdjList.end();
				}
				if ( itr == vAdjList.end() && 
					               colorMap.find(U)->second == GREY ) {
					time = time + 1;
					colorMap[U] = BLACK;
					f[U] = time;
					// For topological sorting
					if (m_Operation == TOPOLOGICALSORT) {
						m_TopologicalSortList.push_front(U);
					}
					S.pop();
				}
			}
	}

public:
	friend std::ostream& operator << (std::ostream& os, graph& graph) {
		std::cout << "-------------------------------------------" << std::endl;
		for ( graph_storage_itr	iter = graph.m_GraphStorage.begin(); 
			iter != graph.m_GraphStorage.end(); 
			++iter) {
				std::cout << *iter << " : " ;
				adj_list_t list = graph.GetAdjList(*iter);
				adj_list_itr iter1;
				for ( iter1 = list.begin(); iter1 != list.end(); ++iter1) {
					std::cout << *(*iter1) << '\t' ;
				}
				std::cout << std::endl;
		}
		std::cout << "-------------------------------------------" << std::endl;
		return os;
	}
private:
	graph_type       m_Type; 
	graph_t          m_Graph;
	edge_vector_t    m_EdgeVector;
	graph_storage_t  m_GraphStorage;
	edge_weight_t    m_EdgeWeight;

	// Distance map
	distance_map_t   m_BFSDistanceMap;
	// Visit Time Map
	TimeMap_t        m_DFSStartTimeMap;
	TimeMap_t        m_DFSFinishTimeMap;
	// Parent Map
	parent_map_t     m_BFSParentMap;
	parent_map_t     m_DFSParentMap;
	// topological sorting list.
	std::list<vertex_t *> m_TopologicalSortList;
	graph_operation m_Operation;

protected:
};


#endif
