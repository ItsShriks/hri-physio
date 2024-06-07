/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory,
 *     University of Waterloo, All rights reserved.
 *
 * Authors:
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 *
 * CopyPolicy: Released under the terms of the BSD 3-Clause License.
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#ifndef HRI_PHYSIO_CORE_GRAPH_H
#define HRI_PHYSIO_CORE_GRAPH_H

#include <algorithm>
#include <cassert>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Core {
        class Graph;

        struct Edge {
            int to;
            int weight; // weight can be substituted for
            Edge(int t, int w) : to(t), weight(w) {}
        };
    }
}

class hriPhysio::Core::Graph {
private:
    /* ============================================================================
    **  Member Variables.
    ** ============================================================================ */
    std::vector<std::vector<Edge>> nbr;
    std::vector<int> dist;
    std::vector<int> prev;

    int num_nodes;
    int num_edges;

public:
    /* ===========================================================================
    **  Main Constructor.
    ** =========================================================================== */
    explicit Graph(int n);

    /* ===========================================================================
    **  Destructor.
    ** =========================================================================== */
    ~Graph() = default;

    /* ===========================================================================
    **  Function that adds an undirected edge from u to v, with weight w.
    **
    ** @param u         ID of the first vertex.
    ** @param v         ID of the second vertex.
    ** @param weight    The cost associated with traveling between these edges.
    **
    ** @return whether or not the addition of this edge was successful.
    ** =========================================================================== */
    bool addEdge(int u, int v, int weight = 1);

    /* ===========================================================================
    **  Function that returns the number of edges in the graph.
    **
    ** @return The number of edges in the graph.
    ** =========================================================================== */
    int getNumEdges() const;

    /* ===========================================================================
    **  Function that finds the shortest path from the source vertex to the target.
    **  The algorithm implemented in Dijkstra's. Path is stored in ``message``.
    **
    ** @param source     ID of the source vertex.
    ** @param target     ID of the target vertex.
    ** @param message    The cost associated with traveling between edges.
    ** =========================================================================== */
    void shortestPath(int source, int target, std::string &message);

private:
    /* ===========================================================================
     *  Complexity is O( (n + m) log (n + m) ) where n is the number of
     *    vertices and m is the number of edges.
     *
     *  See: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
     *
     * @param source    The beginning node to search from.
     * =========================================================================== */
    void dijkstra(int source);
};

#endif // HRI_PHYSIO_CORE_GRAPH_H
