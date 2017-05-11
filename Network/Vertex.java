package Graphs.Network;

import java.util.HashMap;

public class Vertex{

    int index;
    HashMap<Vertex, Edge> neighbors;

    Vertex(int index) {
        this.index = index;
        neighbors = new HashMap<>();

    }

    public int getCapacity(Vertex vertex) {
        return neighbors.get(vertex).capacity;
    }

    public int getIndex() {
        return index;
    }

    void addNeighbor(Vertex v, int capacity){
        neighbors.put(v, new Edge(index, v.index, capacity));
    }

    boolean hasNeighbor(Vertex vertex) {
        return neighbors.get(vertex) != null;
    }

    int getEdgeFlow(Vertex to) {
        return neighbors.get(to).getFlow();
    }

    void updateFlow(Vertex to, int newFlow) {
        neighbors.get(to).updateFlow(newFlow);
    }

    void removeEdge(Vertex v) {
        neighbors.remove(v);
    }

    Edge getEdge(Vertex vertex) {
        return neighbors.get(vertex);
    }


    @Override
    public String toString() {
        return "Vertex index = " + index;
    }
}