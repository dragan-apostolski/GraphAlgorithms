package Graphs.Network;

import java.util.ArrayList;
import java.util.List;

/**
 * This class represents a directed graph as a network. It consists of vertices (represented by {@link Vertex})
 * and edges (represented by {@link Edge} with capacities and flows.
 *
 */
public class Network {

    List<Vertex> adjList;
    private final int numberOfVertices;
    private final int source;
    private final int sink;

    /**
     * Creates an empty network with the given number of vertices and marked source and sink.
     * This initial network contains no edges.
     *
     * @param numberOfVertices the number of vertices
     * @param source the source of the network
     * @param sink the sink of the the network
     */
    public Network(int numberOfVertices, int source, int sink) {
        this.numberOfVertices = numberOfVertices;
        adjList = new ArrayList<>(numberOfVertices);
        for (int i = 0; i < numberOfVertices; i++) {
            adjList.add(i, new Vertex(i));
        }
        this.source = source;
        this.sink = sink;
    }

    /**
     * Adds edge from vertex with index u to vertex with index v with capacity capacity.
     *
     * @param u the index of the first vertex
     * @param v the index of the second vertex
     * @param capacity the capacity of the edge
     */
    public void addEdge(int u, int v, int capacity) {
        adjList.get(u).addNeighbor(adjList.get(v), capacity);
    }

    /**
     * This method computes and returns the maximum flow in this flow network. It is computed with the Ford - Fulkerson
     * method, using a {@link ResidualNetwork} for finding augmenting paths.
     * @return the max flow
     */
    public int maxFlow() {
        fordFulkerson();
        final Vertex source = adjList.get(this.source);
        return source.neighbors.keySet()
                .stream()
                .mapToInt(source::getEdgeFlow)
                .sum();
    }

    public List<Vertex> getAdjList() {
        return adjList;
    }

    /**
     * An implementation of the Ford - Fulkerson algorithm for computing the maximum flow in this network. This method
     * runs in O(E*f) time, where f is the maximum flow in this network.
     */
    private void fordFulkerson(){
        ResidualNetwork residualNetwork = new ResidualNetwork(this, source, sink, numberOfVertices);
        residualNetwork.fordFulkerson();
    }

    void updateFlow(int from, int to, int newFlow) {
        Vertex u = adjList.get(from);
        Vertex v = adjList.get(to);
        u.updateFlow(v, newFlow);
    }

    int getEdgeFlow(int from, int to) {
        Vertex u = adjList.get(from);
        Vertex v = adjList.get(to);
        return u.getEdgeFlow(v);
    }

}