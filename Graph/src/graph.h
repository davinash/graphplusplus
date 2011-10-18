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

#ifndef GRAPH_H__
#define GRAPH_H__

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


enum graph_type {
    DIRECTED,
    UNDIRECTED,
};

enum vertex_color  {
    BLACK,
    WHITE,
    GREY
};

enum graph_op {
    NULL_OP,
    TOPOLOGICAL_SORT
};


template <typename T>
struct  comp_vertex {
    bool operator()(T left,T right) { 
        return *left < *right; 
    }
};

template <typename P>
struct  comp_edge {
    bool operator()(P left,P right) { 
        return (*(left.first) < *(right.first) && 
            *(left.second) < *(right.second)); 
    }
};
// graph template class.
template <typename T, typename W = int >
class graph {
public: 
    // Some typedef for convience.
    typedef typename std::list<T const*>                      adj_list_t;
    typedef typename adj_list_t::iterator                     adj_list_itr_t;

    typedef          comp_vertex<T const*>                    graph_comp_t;
    typedef typename std::map<T const*, adj_list_t, 
                                              graph_comp_t >  graph_t;
    typedef typename graph_t::iterator                        graph_itr_t;

    typedef typename std::set<T>                              storge_t;
    typedef typename storge_t::const_iterator                 storage_itr_t;

    typedef typename std::pair<T const*, T const*>            edge_t ;
    typedef          comp_edge<edge_t>                        comp_edge_t;
    typedef typename std::map<edge_t, W, comp_edge_t >        edge_weight_map_t;

    typedef typename std::map<T const*, vertex_color>         color_map_t;

    typedef typename std::map<T const*, W,  graph_comp_t >    distance_map_t;
    typedef typename std::map<T const*, T const*, 
                                              graph_comp_t >  parent_map_t;

    typedef          distance_map_t                           time_map_t;

    struct bfs_data {
        //The distance from the source s to vertex u 
        //computed by the algorithm is stored in d
        distance_map_t  d;
        //the predecessor of u is stored in the variable p[u].
        parent_map_t    p;
        // Stores the order in which vertices are visited
        std::vector<T const*> visitedVec; 

        bfs_data() {
            d.clear();
            p.clear();
            visitedVec.clear();
        }
        ~bfs_data() {
            d.clear();
            p.clear();
            visitedVec.clear();
        }
        // Iterator definition for bfs data 
        typedef typename std::vector<T const*>::iterator iterator;
        typedef typename std::vector<T const*>::const_iterator const_iterator;

	    iterator begin() {
		    return (visitedVec.begin());
		}
	    const_iterator begin() const {
		    return (visitedVec.begin());
		}
	    iterator end() {
		    return (visitedVec.end());
		}
	    const_iterator end() const {
		    return (visitedVec.end());
		}
    };
    struct dfs_data {
         //records when it discovers vertex u in the variable d[u]
        time_map_t      d;
        // records when it finishes vertex u in the variable f [u]. 
        time_map_t      f;
        //the predecessor of u is stored in the variable p[u].
        parent_map_t    p;
        // Stores the order in which vertices are visited
        std::vector<T const*> visitedVec; 
        // stores the topological order of vertices
        std::list<T const*> topList;
        dfs_data() {
            d.clear();
            f.clear();
            p.clear();
            visitedVec.clear();
            topList.clear();
        }
        ~dfs_data() {
            d.clear();
            f.clear();
            p.clear();
            visitedVec.clear();
            topList.clear();
        }
        // Iterator definition for dfs data 
        typedef typename std::vector<T const*>::iterator iterator;
        typedef typename std::vector<T const*>::const_iterator const_iterator;
	    iterator begin() {
		    return (visitedVec.begin());
		}
	    const_iterator begin() const {
		    return (visitedVec.begin());
		}
	    iterator end() {
		    return (visitedVec.end());
		}
	    const_iterator end() const {
		    return (visitedVec.end());
		}
    };

public:
    // Construct object of graph type.
    // Accept type of the graph
    // Available type are
    // 1. Undirected graph
    // 2. Directed graph
    // Default graph type is Undirected.
    graph(graph_type eType = UNDIRECTED) : m_type(eType), 
                                           m_bIs_weighted(false) {
    }
    // destructor
    // Does not call destructor for each of the elements in the container
    // It is important the user of this calls destructor for each of the 
    // element explicitly.
    ~graph() {
        m_storage.clear();
    }
    // insert vertex in the graph
    // if the vertex is already in the graph
    // function just return and does nothing.
    // also initialize the Adjacency list of the vertex.
    void insert ( T vertex ) {
        if ( find_vertex_address(vertex) == m_storage.end()) {
            // insert the vertex data in the graph storage
            // this the only place where actual vertex is stored
            // all other places, address of this will be used.
            m_storage.insert(vertex);
            // insert the vertex address from storage into the 
            // graph and intiailize empty adjacency list.
            m_graph[&*(find_vertex_address(vertex))] = adj_list_t();
        }
    }
    // insert
    // inserts an edge in the graph
    void insert(T left , T right) {
        insert(left);
        insert(right);
        // Get the address from storage space.
        T const* vL  = &*(find_vertex_address(left));
        T const* vR  = &*(find_vertex_address(right));

        m_graph[vL].push_back(vR);
        if ( m_type == UNDIRECTED) {
            m_graph[vR].push_back(vL);
        }
    }
    // insert add edge to the graph with the weight
    void insert( T left, T right, W weight) {
        insert(left, right );
        // Get the address from the storage-space
        T *vL  = &*(find_vertex_address(left));
        T *vR = &*(find_vertex_address(right));
        m_edge_weight_map.insert(std::make_pair(std::make_pair(vL,vR),weight));
        m_bIs_weighted = true;
    }
    // performs breadth first search on the graph.
    // Returns pointers to the bfs_data structure
    // User must delete bfs_data delete
    bfs_data* breadh_first_search( T* inS = (T*)0) {
        bfs_data *pbd = new bfs_data();
        color_map_t color;

        T const* S = const_cast<T*>(inS);

        if (!S ) {
            S = m_graph.begin()->first;
        }
        for ( graph_itr_t itr = m_graph.begin();itr != m_graph.end(); ++itr) {
            T const *U        = itr->first;
            color[U]    = WHITE;    //paint every vertex white
            pbd->d[U]   = INT_MAX;  //set d[u] to be infinity for each vertex u
            pbd->p[U]   = (T*)0;    //set the parent of every vertex to be NIL
        }
        color[S]    = GREY;
        pbd->d[S]   = 0;
        pbd->p[S]   = (T*)0;

        std::deque<T const*> Q;
        Q.push_back(S);//initialize Q to the queue containing just vertex S. 
        //iterates as long as there remain gray vertices, 
        //which are discovered vertices that have not yet 
        //had their adjacency lists fully examined.
        while ( !Q.empty()) {
            T const* U  = Q.front();
            Q.pop_front();
            adj_list_t adj_list = m_graph.find(U)->second;
            for ( adj_list_itr_t it = adj_list.begin(); it != adj_list.end(); 
                ++it) {
                T const *V = *(it);
                if ( color.find(V)->second == WHITE ) {
                    color[V]   = GREY;
                    pbd->d[V]  = pbd->d[U] + 1;
                    pbd->p[V]  = U;

                    Q.push_back(V);
                }
            }
            color[U] = BLACK;
            pbd->visitedVec.push_back(U);
        }
        return pbd;
    }
    // performs depth first search on the graph.
    // Returns pointers to the bfs_data structure
    // User must delete dfs_data delete
    dfs_data* depth_first_search(graph_op op = NULL_OP) {
        color_map_t color;
        dfs_data* pdd = new dfs_data();

        for ( graph_itr_t it = m_graph.begin();it != m_graph.end(); ++it) {
            color[it->first] = WHITE;//paint all vertices white 
            pdd->p[it->first] = (T*)0; //initialize their parent fields to NIL
        }
        //global time counter
        int time = 0;
        // check each vertex in V in turn and, when a white vertex is found, 
        // visit it using DFS-VISIT
        for ( graph_itr_t it = m_graph.begin();it != m_graph.end(); ++it) {
            if ( color.find(it->first)->second == WHITE ) {
                DFS_Visit(it->first, pdd, time, color, op);
            }
        }
        return pdd;
    }
    //1  call DFS(G) to compute finishing times f[v] for each vertex v
    //2  as each vertex is finished, insert it onto the front of a linked list
    //3  return the linked list of vertices
    dfs_data* topological_sort() {
        return depth_first_search(TOPOLOGICAL_SORT);
    }
private:
    storage_itr_t find_vertex_address(T vertex) const {
        return m_storage.find(vertex);
    }
    void DFS_Visit( T const* inU, dfs_data* pdd , int& time, 
                    color_map_t& color, graph_op op) {
        std::stack<T const*> S;
        S.push(inU);
        while ( !S.empty()) {
            T const * U  = S.top();                         
            adj_list_t vAdjList = m_graph.find(U)->second;
            adj_list_itr_t itr;
            if ( color.find(U)->second == WHITE ) {  
                time = time + 1;
                pdd->d[U] = time;
                color[U] = GREY;
                for ( itr = vAdjList.begin(); itr != vAdjList.end(); ++itr){
                    T const* V = *(itr);
                    if ( color.find(V)->second == WHITE ) {
                        pdd->p[V] = U;
                        if ( op == NULL_OP) {
                            S.push(V);
                        }
                        break;
                    }
                }
            } else {
                itr = vAdjList.end();
            }
            if ( itr == vAdjList.end() && 
                color.find(U)->second == GREY ) {
                time = time + 1;
                color[U] = BLACK;
                pdd->f[U] = time;
                if ( op == TOPOLOGICAL_SORT) {
                    pdd->topList.push_front(U);
                }
                S.pop();
            }
        }
    }
private:
    graph_type         m_type; // 1. directed or un-directed 
    adj_list_t         m_adj_list;
    graph_t            m_graph;
    storge_t           m_storage;
    edge_weight_map_t  m_edge_weight_map;
    bool               m_bIs_weighted;

};

#endif