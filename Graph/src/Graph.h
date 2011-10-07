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

// To avoid warning C4503:
// decorated name length exceeded, name was truncated
#pragma warning(disable:4503)

enum VertexColor  {
    BLACK,
    WHITE,
    GREY
};

enum GraphType {
    DIRECTED,
    UNDIRECTED
};

enum GraphOperation {
    TOPOLOGICALSORT
};

template <typename UF>
class UnionFind {
private:
    struct Element {
        UF m_value;
        int m_parent;
        int m_rank;

        Element(const UF& value, int parent)
        :   m_value(value), m_parent(parent), m_rank(0)
        {}
    };
private:
    std::map<int,Element> m_Elements;
public:
    UnionFind(){}
public:
    void AddElement(int x, const UF& value = UF()) {
        m_Elements.insert(std::make_pair(x, Element(value, x)));
    }
    int FindSet(int x) {
        Element& element = GetElement(x);
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
    Element& GetElement(int x) {
        typename std::map<int,Element>::iterator it = m_Elements.find(x);
		return it->second;
    }

    void SetupParentRelationship(int x, int y) {
        Element& elementX = GetElement(x);
        Element& elementY = GetElement(y);
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


template <typename TypeV>
class Vertex {
public:
    Vertex(TypeV in) :	m_Data(in){ }
    ~Vertex() { }
    Vertex( Vertex const &right) : m_Data(right.m_Data){}

    bool operator < ( const Vertex & right) const {
        return m_Data < right.m_Data;
    }

    bool operator == ( const Vertex & right ) const {
        return m_Data == right.m_Data;
    }

    friend std::ostream& operator << (std::ostream& os, 
                                const Vertex& vertex) {
        return os << vertex.m_Data;	
    }
    TypeV getRemoteData() { return m_Data;}
private:
    TypeV m_Data;
public:
    VertexColor m_Color;
protected:
};

template <typename TypeI>
struct  CompareIterator {
      bool operator()(TypeI left,TypeI right) { 
          return *left < *right; 
      }
};

template <typename TypeG>
class Graph {
private:		
public:
    typedef typename Vertex<TypeG>                      Vertex_t;

    typedef typename std::set<Vertex_t>                 GraphStorage;
    typedef typename GraphStorage::iterator             GraphStorageItr;

    typedef typename std::list<Vertex_t *>              AdjList_t;
    typedef typename AdjList_t::iterator                AdjListItr_t;

    typedef typename std::map<Vertex_t*, 
                              AdjList_t, 
                              CompareIterator<Vertex_t*> > Graph_t;
    typedef typename Graph_t::iterator      GraphTypeItr;

    typedef typename std::map<Vertex_t*, 
                              int, 
                              CompareIterator<Vertex_t*> > DistanceMap_t;
    typedef typename DistanceMap_t TimeMap_t;
    typedef typename std::map<Vertex_t*, 
                              Vertex_t*, 
                              CompareIterator<Vertex_t*> > ParentMap_t;
	typedef typename std::vector<std::pair<int, std::pair<Vertex_t*, Vertex_t*> > > Edge_t;
	typedef typename Edge_t::iterator EdgeIter_t;

public:
    Graph(GraphType eType = UNDIRECTED) : m_Type(eType) {
        m_TopologicalSortList = std::list<Vertex_t *>();
    }
    ~Graph() {}

    void AddVertex(Vertex_t vertexID) {
        GraphStorageItr pos = m_GraphStorage.find(vertexID);
        if ( pos == m_GraphStorage.end()) {
            m_GraphStorage.insert(vertexID);
            m_Graph[&(*(m_GraphStorage.find(vertexID)))] = AdjList_t();
        }	
    }

    void AddVertex(TypeG vVertex) {
        this->AddVertex((Vertex_t) vVertex);
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
    }

    void AddEdge(TypeG vLeft, TypeG vRight) {
        this->AddEdge((Vertex_t) vLeft, (Vertex_t) vRight);
    }

    void AddEdge(TypeG vLeft, TypeG vRight, int weight) {
        this->AddEdge((Vertex_t) vLeft, (Vertex_t) vRight);

		Vertex_t *vLeftPtr  = &(*(m_GraphStorage.find(vLeft)));
        Vertex_t *vRightPtr = &(*(m_GraphStorage.find(vRight)));

		m_EdgeVector.push_back(std::make_pair(weight, 
                                              std::make_pair(vLeftPtr,
                                                             vRightPtr)));
    }

    void BFS(Vertex_t *pStart = (Vertex_t*)0) {
        if ( ! pStart) {
            pStart = m_Graph.begin()->first;
        }
        _BFS(pStart);

    }

    void DFS(Vertex_t *pStart = (Vertex_t*)0) {
        if ( ! pStart) {
            pStart = m_Graph.begin()->first;
        }
        _DFS(pStart,0);
    }


    bool GetShortestPath(TypeG S, TypeG V, std::vector<TypeG> &outPath) {

        Vertex_t *pS  = (Vertex_t*)0;
        Vertex_t *pV =  (Vertex_t*)0;

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

    bool SortToplogical( std::list<TypeG> &outList) {
        m_Operation = TOPOLOGICALSORT;
        _DFS(m_Graph.begin()->first);
        for ( std::list<Vertex_t *>::iterator itr = m_TopologicalSortList.begin();
            itr != m_TopologicalSortList.end(); ++itr) {
                outList.push_back((*itr)->getRemoteData());
        }
        m_TopologicalSortList.clear();
        return true;
    }

	void BuildMST() {
		UnionFind<Vertex_t*> ufSet;
		int index = 0;
		std::map<Vertex_t*, int> VertexArray;
		//MAKE-SET(v)
		for ( GraphTypeItr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
            Vertex_t *U = itr->first;
			ufSet.AddElement(index, U);
			VertexArray.insert(std::make_pair(U,index));
			index++;
		}
		//sort the edges of E into nondecreasing order by weight w
		std::sort(m_EdgeVector.begin(), m_EdgeVector.end());
		for ( EdgeIter_t it = m_EdgeVector.begin(); it != m_EdgeVector.end(); ++it) {
			Vertex_t *pLeft  = (it->second).first;
			Vertex_t *pRight = (it->second).second;
			int leftIndex    = (VertexArray.find(pLeft))->second;
			int rightIndex   = (VertexArray.find(pRight))->second;
			if ( ufSet.FindSet(leftIndex) != ufSet.FindSet(rightIndex)){
				ufSet.UnionSets(leftIndex,rightIndex);
			}
		}
	}
    
private:
    bool _GetShortestPath(Vertex_t *S, Vertex_t *V, std::vector<TypeG> & pathVector){
        if (*S == *V) {
            pathVector.push_back(V->getRemoteData());
        } else {
            if ( m_BFSParentMap[V] == ( Vertex_t*)0)
                return false;
            else
                _GetShortestPath(S,m_BFSParentMap[V],pathVector);

            pathVector.push_back(V->getRemoteData());
        }
        return true;
    }
    void _BFS(Vertex_t *S) {
        // 1. Paint every vertex as WHITE
        // 2. Set the d to -1
        // 3. Set the p of each vertex to NULL
        m_BFSDistanceMap.clear();
        m_BFSParentMap.clear();
        for ( GraphTypeItr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
            Vertex_t *U = itr->first;
            U->m_Color =  WHITE;
            m_BFSDistanceMap[U] = -1;
            m_BFSParentMap[U] = (Vertex_t *)0;
        }
        S->m_Color = GREY;
        // Initialize the d to 0
        m_BFSDistanceMap[S] = 0;
        m_BFSParentMap[S] = (Vertex_t *)0;

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
                    V->m_Color = GREY;
                    m_BFSDistanceMap[V] = m_BFSDistanceMap[U] + 1;
                    m_BFSParentMap[V] = U;
                    Q.push_back(V);
                }
            }
            U->m_Color = BLACK;
        } // Queue empty loop
    }
    void _DFS(Vertex_t *S) {
        m_DFSStartTimeMap.clear();
        m_DFSFinishTimeMap.clear();
        m_DFSParentMap.clear();		
        for ( GraphTypeItr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
            itr->first->m_Color        = WHITE;
            m_DFSParentMap[itr->first] = (Vertex_t*)0;
        }
        int time = 0;
        for ( GraphTypeItr itr = m_Graph.begin();itr != m_Graph.end(); ++itr) {
            if ( itr->first->m_Color == WHITE ) {
                _DFS_Visit(	itr->first,
                            m_DFSStartTimeMap,
                            m_DFSFinishTimeMap,
                            m_DFSParentMap, 
                            time);
            }
        }
    }
    void _DFS_Visit(Vertex_t *inU, TimeMap_t & d, 
                    TimeMap_t & f, ParentMap_t & p , 
                    int & time) {
            std::stack<Vertex_t *> S;
            S.push(inU);
            while ( !S.empty()) {
                Vertex_t *U  = S.top();				
                AdjList_t vAdjList = m_Graph.find(U)->second;
                AdjListItr_t itr;
                if ( U->m_Color == WHITE ) {  
                    time = time + 1;
                    d[U] = time;
                    U->m_Color = GREY;
                    for ( itr = vAdjList.begin(); itr != vAdjList.end(); ++itr) {
                        Vertex_t *V = *(itr);
                        if ( V->m_Color == WHITE ) {
                            p[V] = U;
                            S.push(V);
                            break;
                        }
                    }
                } else {
                    itr = vAdjList.end();
                }
                if ( itr == vAdjList.end() && U->m_Color == GREY ) {
                    time = time + 1;
                    U->m_Color = BLACK;
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
	Edge_t    m_EdgeVector;
    GraphStorage  m_GraphStorage;

    // Distance map
    DistanceMap_t  m_BFSDistanceMap;
    // Visit Time Map
    TimeMap_t      m_DFSStartTimeMap;
    TimeMap_t      m_DFSFinishTimeMap;
    // Parent Map
    ParentMap_t    m_BFSParentMap;
    ParentMap_t    m_DFSParentMap;
    // topological sorting list.
    std::list<Vertex_t *> m_TopologicalSortList;
    GraphOperation m_Operation;

protected:
};


#endif
