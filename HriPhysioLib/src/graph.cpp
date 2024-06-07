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

#include <HriPhysio/Core/graph.h>
#include <stdexcept>

using namespace hriPhysio::Core;


Graph::Graph(int n) : num_nodes(n), num_edges(0), nbr(std::make_unique<std::vector<Edge>[]>(num_nodes)) {
    for (auto& edges : nbr) {
        edges.clear();
    }
}


Graph::~Graph() = default;

void Graph::addEdge(int u, int v, double weight/*=1.0*/) {
    if (u >= num_nodes || u < 0 || v >= num_nodes || v < 0) {
        throw std::out_of_range("Node index out of range");
    }
    //-- Add an ``undirected`` edge between the vertices.
    nbr[u].push_back(Edge(v, weight));
    nbr[v].push_back(Edge(u, weight));
    ++num_edges;
}


int Graph::getNumEdges() const {
    return num_edges;
}


void Graph::shortestPath(int source, int target, std::string& message) {

    //-- Check to see if source and target are the same.
    if (source == target) {
        message = std::to_string(source) + '-' + std::to_string(target);
        return;
    }

    //-- Run dijkstras algorithm.
    dijkstra(source);

    //-- Recover the path from the previous 
    //-- nodes used to get to the target.
    std::vector<int> path;
    for (int from = target; from != -1; from = prev[from]) {
        path.push_back(from);
    }
    //-- Reverse this so constructing the string is easier.
    std::reverse(path.begin(), path.end());
    //-- If there was only one element in the path, 
    //-- then no such route exists.
    if (path.size() == 1) {
        message.clear();
        return;
    }
    message.clear();
    //-- Construct the string adding ``-`` between the nodes.
    for (size_t idx = 0; idx < path.size(); ++idx) {
        if (idx !=0) {
            message += "-";
        }           //-- i.e. not zero.
        message += std::to_string(path[idx]);  //-- append the node to the message.
    }
}


void Graph::dijkstra(int source) {

    //-- Keep track of which nodes have been used.
    std::vector< bool > used(num_nodes, false);

    //-- Use a p-queue to maintaine least-cost path.
    std::priority_queue< pii, std::vector<pii>, std::greater<pii> > fringe;

    //-- Flush some buffers we will use.
    dist.assign(num_nodes, -1);
    prev.assign(num_nodes, -1);
    //-- Init the first ``step``.
    dist[source] = 0;
    fringe.emplace(dist[source], source);


    //-- Take steps in the graph until a path is found!
    //-- If there is no path, we will exhaust all options.
    while (!fringe.empty()) {
        auto [currentDist, from] = fringe.top();
        fringe.pop();

        //-- If we have already tried to use this vertex, don't bother.
        if (used[from]) {
            continue;
        }

        //-- Don't try to revisit here in the future.
        used[from] = true;

        //-- Check each neighbor from u to v.
        for (const auto& edge : nbr[from]) {
            int to = edge.to;
            int weight = edge.weight + currentDist;

            if (used[to]) {
                continue;
            }
            //-- If this node has not been visited yet, or if 
            //-- we have found a better path to get here, update.
            if (dist[to] == -1 || weight < dist[to]) {

                dist[to] = weight;
                prev[to] = from;

                fringe.emplace( dist[to], to) );
            }
        }
    }
}
