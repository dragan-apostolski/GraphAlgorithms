package Graphs.DirectedWeighted;

import java.util.HashMap;


public class Vertex implements Comparable<Vertex>,Cloneable{
    public int index;
    public HashMap<Vertex, Double> neighbors;

    Vertex(int index) {
        this.index = index;
        distance = Double.POSITIVE_INFINITY;
        neighbors = new HashMap<>();
    }

    public int getIndex() {
        return index;
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

    double getWeight(Vertex v){
        Double d = neighbors.get(v);
        return (d != null) ? d : Double.POSITIVE_INFINITY;
    }


    /**
     * The field distance in this class represents the distance of this vertex from
     * an adjacent vertex in the process of discovering the shortest path from some
     * source vertex to some end vertex in the graph which contains this vertex.
     */
    private Double distance;

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public Double getDistance() {
        return distance;
    }


    /**
     * Parameters needed for doing a topological sort and depth-first search of a graph
     * which contains this vertex.
     * timeDiscovered stands for the first time this vertex is discovered by a dfs visit, and
     * timeFinished stands for the last time this vertex is dfs visited. Time is a local field
     * in the {@link DirectedGraph} class, which is used the depth-first search method and always
     * starts off at zero, increasing each time a new vertex is discovered or it is visited for
     * the last time.
     */
    int timeDiscovered;
    int timeFinished;

    public int getTimeDiscovered() {
        return timeDiscovered;
    }

    public int getTimeFinished() {
        return timeFinished;
    }

    @Override
    public int compareTo(Vertex o) {
        return distance.compareTo(o.distance);
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        Object clone = super.clone();
        Vertex v = new Vertex(this.index);
        v.distance = distance;
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