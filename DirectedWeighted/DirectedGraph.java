package Graphs.DirectedWeighted;

import java.util.*;

/** A class that represents a directed graph with weighted edges.
 * Every vertex is represented via the {@link Vertex} class, while every edge is
 * internally represented in the same Vertex class.
 *
 * @author drapostolski
 */
public class DirectedGraph implements Cloneable{

    protected int numberVertices;
    protected Vertex[] adjList;


    /**Creates an empty Directed Graph with the given number of vertices. No edges are created.
     *
     * @param numberOfVertices the initial number of vertices
     */
    public DirectedGraph(int numberOfVertices) {
        this.numberVertices = numberOfVertices;
        adjList = new Vertex[numberOfVertices];
        for(int i = 0; i < numberOfVertices; i++)
            adjList[i] = new Vertex(i);
    }


    /**Adds a directed edge (u, v) with the given weight to the graph.
     *
     * @param u the index of the vertex where the edge comes out from
     * @param v the index of the vertex where the edge is directed to
     * @param weight the weight of the directed edge (u, v)
     */
    public void addEdge(int u, int v, int weight){
        adjList[u].addNeighbor(adjList[v], weight);
    }


    /**Removes the directed edge (u,v) from the graph.
     *
     * @param u the index of the source vertex
     * @param v the index of the end vertex
     */
    public void removeEdge(int u, int v){
        adjList[u].removeNeighbor(adjList[v]);
    }

    public void addVertex(Vertex v){
        adjList = Arrays.copyOf(adjList, adjList.length + 1);
        adjList[adjList.length - 1] = v;
        numberVertices = adjList.length;
    }


    /** Returns the weight of the directed edge between u and v.
     * @param u the index of the source vertex
     * @param v the index of the end vertex
     * @return the weight of the edge (u, v).
     */
    protected double weight(Vertex u, Vertex v){
        return adjList[u.index].neighbors.get(v);
    }

    protected void relax(double distance [], Vertex u, Vertex v){
        if (distance[u.index] + weight(u, v) < distance[v.index]) {
            distance[v.index] = distance[u.index] + weight(u, v);
        }
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        Object clone = super.clone();
        DirectedGraph g = new DirectedGraph(numberVertices);
        for (int i = 0; i < g.adjList.length; i++) {
            g.adjList[i] = (Vertex) adjList[i].clone();
        }
        return g;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        Arrays.stream(adjList).forEach(sb::append);
        return sb.toString();
    }

    /** An implementation of Dijkstra's algorithm with a binary min {@link PriorityQueue}
     * that returns the shortest paths from the source vertex to every other vertex that can
     * be reached.
     * The predecessor array in this method consists of the predecessors of each vertex in the shortest
     * path from the source vertex to that vertex. Since this method computes the shortest paths from
     * the source vertex to every other vertex, that array makes no use. But it can be easily modified
     * to print the shortest path if this method is overridden to compute the shortest path from a source
     * vertex to a given end vertex.
     *
     * The running time of this method is O((V + E) * ln(V)), where V denotes the number of vertices
     * and E denotes the number of edges in the graph.
     *
     * @param source the index of the starting vertex
     * @return an array with the shortest paths reachable from the source vertex
     */
    public double [] dijkstra(int source){
        double [] distance = new double[numberVertices];
        int [] predecessor = new int[numberVertices];
        for (int i = 0; i < numberVertices; i++) {
            distance[i] = Double.POSITIVE_INFINITY;
            predecessor[i] = -1;
        }
        distance[source] = 0;
        predecessor[source] = source;
        PriorityQueue<Vertex> pq = new PriorityQueue<>();
        pq.add(adjList[source]);
        while(!pq.isEmpty()){
            Vertex u = pq.remove();
            for (Vertex v : u.neighbors.keySet()) {
                if(distance[u.index] + weight(u, v) < distance[v.index]){
                    distance[v.index] = distance[u.index] + weight(u, v);
                    v.setMinDistance(distance[v.index]);
                    pq.add(v);
                    predecessor[v.index] = u.index;
                }
            }
        }

        return distance;
    }


    /** An implementation of the Bellman - Ford algorithm for shortest paths in a graph
     * with negative edge weights.
     * This algorithm computes the shortest paths from the source vertex and checks
     * if a negative weight cycle is reachable from the source vertex. If so, the method throws
     * a {@link NegativeWeightCycleException}, indicating that shortest paths can't be computed
     * from this source vertex. If the graph is strongly connected and has a negative weight cycle,
     * a call of this method from any source vertex will always result in a {@link NegativeWeightCycleException}.
     *
     * This method runs in O(V*E) time, where V is the number of vertices and E is
     * the number of edges in the graph
     *
     * @param source the index of the source vertex
     * @return a double array with the shortest paths from the source vertex to
     * every other reachable vertex in the graph
     * @throws NegativeWeightCycleException if a negative weight cycle is detected
     */
    public double [] bellmanFord(int source) throws NegativeWeightCycleException {
        double[] distance = new double[numberVertices];
        for (int i = 0; i < distance.length; i++) {
            distance[i] = Double.POSITIVE_INFINITY;
        }
        distance[source] = 0;
        for (int i = 0; i < numberVertices - 1; i++) {
            for (Vertex u : adjList) {
                for (Vertex v : u.neighbors.keySet()) {
                    relax(distance, u, v);
                }
            }
        }
        for (Vertex u : adjList) {
            for (Vertex v : u.neighbors.keySet()) {
                if (distance[u.index] + weight(u, v) < distance[v.index]) {
                    throw new NegativeWeightCycleException("Negative weight cycle detected");
                }
            }
        }
        return distance;
    }

    /** An implementation of Johnson's algorithm for constructing the all - pair shortest paths matrix in
     * a sparse graph. This method depends on the Bellman-Ford implementation to detect negative weight cycles,
     * after the graph structure is augmented with adding a new source vertex. If such cycle is detected, the
     * {@link NegativeWeightCycleException} is rethrown from the bellmanFord method. It also depends on the
     * Dijkstra's algorithm implementation, now being assured that there are no negative weight cycles,
     * to run it from each source vertex in the process of computing the all - pair shortest paths.
     *
     * This method runs in O(V*E*lgV) time (given that dijkstra is implemented with a binary {@link PriorityQueue})
     * and is a slightly faster than the Floyd - Warshall implementation in the {@link Graphs.DirectedMatrixGraph.Graph}
     * class. It is a better choice to use this method to compute the all - pair shortest paths if the graph is sparse,
     * that is it has fewer edges, than running the Floyd - Warshall implementation or the matrix multiplication method.
     *
     * @return the all - pair shortest paths matrix
     * @throws CloneNotSupportedException (this shouldn't happen)
     * @throws NegativeWeightCycleException if a negative weight cycle is detected
     */
    public Double [][] johnson() throws CloneNotSupportedException, NegativeWeightCycleException {
        DirectedGraph graph = (DirectedGraph) this.clone();
        Vertex s = new Vertex(graph.adjList.length);
        for (Vertex vertex : this.adjList) {
            s.addNeighbor(vertex, 0);
        }
        graph.addVertex(s);
        Map<Vertex, Double> h = new HashMap<>();
        double [] sp = graph.bellmanFord(s.index);
        for (int i = 0; i < sp.length - 1; i++) {
            h.put(adjList[i], sp[i]);
        }
        for (int i = 0; i < numberVertices; i++) {
            Vertex u = this.adjList[i];
            for (Vertex v : u.neighbors.keySet()) {
                u.updateWeight(v, weight(u, v) + h.get(u) - h.get(v));
            }
        }
        Double [][] D = new Double[numberVertices][numberVertices];
        for (int i = 0; i < D.length; i++) {
            double [] sps = dijkstra(i);
            for (int j = 0; j < D.length; j++) {
                D[i][j] = sps[j] + h.get(adjList[j]) - h.get(adjList[i]);
            }
        }
        return D;
    }
}