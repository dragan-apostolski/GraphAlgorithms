package Graphs.DirectedWeighted;

import java.util.HashMap;


public class Vertex implements Comparable<Vertex>,Cloneable{
    public int index;
    private Double minDistance;
    public java.util.HashMap<Vertex, Double> neighbors;

    Vertex(int index) {
        this.index = index;
        minDistance = Double.POSITIVE_INFINITY;
        neighbors = new HashMap<>();
    }

    public void setMinDistance(double distance) {
        this.minDistance = distance;
    }

    public Double getMinDistance() {
        return minDistance;
    }

    void addNeighbor(Vertex v, double weight){
        neighbors.put(v, weight);
    }

    void removeNeighbor(Vertex v){
        neighbors.remove(v);
    }

    void updateWeight(Vertex u, double newWeight){
        neighbors.put(u, newWeight);
    }

    @Override
    public int compareTo(Vertex o) {
        return minDistance.compareTo(o.minDistance);
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        Vertex v = new Vertex(this.index);
        v.minDistance = minDistance;
        v.neighbors = new HashMap<>(neighbors);
        return v;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Index: ").append(index).append("\n");
        sb.append("Neighbors: (index : weight)").append("\n");
        for (Vertex vertex : neighbors.keySet()) {
            sb.append(vertex.index).append(" : ").append(neighbors.get(vertex)).append("\n");
        }
        sb.append("\n");
        return sb.toString();
    }
}