package Graphs.DirectedWeighted;

import com.intellij.util.containers.HashMap;


public class Vertex implements Comparable<Vertex>{
    public int index;
    private Integer minDistance;
    public HashMap<Vertex, Integer> neighbors;

    Vertex(int index) {
        this.index = index;
        minDistance = Integer.MAX_VALUE;
        neighbors = new HashMap<>();
    }

    public void setMinDistance(int weight) {
        this.minDistance = weight;
    }


    void addNeighbor(Vertex v, int weight){
        neighbors.put(v, weight);
    }

    void removeNeighbor(Vertex v){
        neighbors.remove(v);
    }

    @Override
    public int compareTo(Vertex o) {
        return minDistance.compareTo(o.minDistance);
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