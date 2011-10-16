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
  UNDIRECTED
};

enum vertex_color  {
  BLACK,
	WHITE,
	GREY
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
  typedef typename std::list<T*>                             adj_list_t;
  typedef typename adj_list_t::iterator                      adj_list_itr_t;

  typedef typename comp_vertex<T*>                           graph_comp_t;
  typedef typename std::map<T*, adj_list_t, graph_comp_t >   graph_t;
  typedef typename graph_t::iterator                         graph_itr_t;

  typedef typename std::set<T>                               storge_t;
  typedef typename storge_t::iterator                        storage_itr_t;

  typedef typename std::pair<T*, T*>                         edge_t ;
  typedef typename comp_edge<edge_t>                         comp_edge_t;
	typedef typename std::map<edge_t, W, comp_edge_t >         edge_weight_map_t;

  typedef typename std::map<T*, vertex_color>                color_map_t;

  typedef typename std::map<T*, W,  graph_comp_t >           distance_map_t;
	typedef typename std::map<T*, T*, graph_comp_t >           parent_map_t;
public:
  // Construct object of graph type.
  // Accept type of the graph
  // Available type are
  // 1. Undirected graph
  // 2. Directed graph
  // Default graph type is Undirected.
  graph(graph_type eType = UNDIRECTED) : m_type(eType) {
    m_intermediate_values.clear();
    }
  // destructor
  // Does not call destructor for each of the elements in the container
  // It is important the user of this calls destructor for each of the 
  // element explicitly.
  ~graph() {
    m_intermediate_values.clear();
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

    T *vL  = &*(find_vertex_address(left));
    T *vR  = &*(find_vertex_address(right));

    m_graph[vL].push_back(vR);
    if ( m_type == UNDIRECTED) {
      m_graph[vR].push_back(vL);
		}
  }
  // insert add edge to the graph with the weight
  void insert( T left, T right, W weight) {
    insert(left, right );
    T *vL  = &*(find_vertex_address(left));
    T *vR = &*(find_vertex_address(right));
    m_edge_weight_map.insert(std::make_pair(std::make_pair(vL,vR),weight));
  }
  void breadh_first_search(T* S = (T*)0, bool internal = false) {
    m_intermediate_values.clear();
    m_bfs_distance_map.clear();
    m_bfs_parent_map.clear();

    color_map_t col_map;

    if (!S ) {
      S = m_graph.begin()->first;
    }
    for ( graph_itr_t itr = m_graph.begin();itr != m_graph.end(); ++itr) {
      T *U                  = itr->first;
      col_map[U]            = WHITE;
      m_bfs_distance_map[U] = -1;
      m_bfs_parent_map[U]   = (T*)0;
		}
    col_map[S]              = GREY;
    m_bfs_distance_map[S]   = 0;
    m_bfs_parent_map[S]     = (T*)0;

		std::deque<T*> Q;
		Q.push_back(S);
		while ( !Q.empty()) {
			T* U  = Q.front();
			Q.pop_front();
			adj_list_itr_t adj_list = m_graph.find(U)->second;
			/*for ( adj_list_itr_t it = adj_list.begin(); it != adj_list.end(); ++it) {
				T *V = *(it);
				if ( col_map.find(V)->second == WHITE ) {
					col_map[V]             = GREY;
					m_bfs_distance_map[V]  = m_bfs_distance_map[U] + 1;
					m_bfs_parent_map[V]    = U;
					Q.push_back(V);
				}
			}*/
			col_map[U] = BLACK;
		}
  }
private:
  storage_itr_t find_vertex_address(T vertex) {
    return m_storage.find(vertex);
  }
private:
  graph_type m_type; // 1. directed or un-directed
  // holds the intermediates values and will be used by mainly by
  // iterator interface.
  std::vector<T*>    m_intermediate_values; 
  adj_list_t         m_adj_list;
  graph_t            m_graph;
  storge_t           m_storage;
  edge_weight_map_t  m_edge_weight_map;

  distance_map_t     m_bfs_distance_map;
  parent_map_t       m_bfs_parent_map;
};

#endif