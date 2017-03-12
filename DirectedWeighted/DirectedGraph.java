package Graphs.DirectedWeighted;

import java.util.*;

/** A class that represents a directed graph with weighted edges.
 * Every vertex is represented via the {@link Vertex} class, while every edge is
 * internally represented in the same Vertex class.
 *
 * @author drapostolski
 */
public class DirectedGraph {

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
    @SuppressWarnings("unused")
    public void removeEdge(int u, int v){
        adjList[u].removeNeighbor(adjList[v]);
    }


    /** Returns the weight of the directed edge between u and v.
     * @param u the index of the source vertex
     * @param v the index of the end vertex
     * @return the weight of the edge (u, v).
     */
    protected int weight(Vertex u, Vertex v){
        return adjList[u.index].neighbors.get(v);
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
     * the source vertex to every other vertex, that method makes no use. But it can be easily modified
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
        @SuppressWarnings("MismatchedReadAndWriteOfArray")
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
     * if a negative cycle is reachable from the source vertex. If so, the method throws
     * a {@link NegativeCycleException}, indicating that shortest paths can't be computed
     * from this source vertex. If the graph is strongly connected and has a negative cycle,
     * a call of this method from any source vertex will always throw a {@link NegativeCycleException}.
     *
     * This method runs in O(V*E) time, where V is the number of vertices and E is
     * the number of edges in the graph
     *
     * @param source the index of the source vertex
     * @return a double array with the shortest paths from the source vertex to
     * every other reachable vertex in the graph
     * @throws NegativeCycleException if a negative cycle is detected
     */

    @SuppressWarnings("WeakerAccess")
    public double [] bellmanFord(int source) throws NegativeCycleException {
        double[] distance = new double[numberVertices];
        for (int i = 0; i < distance.length; i++) {
            distance[i] = Double.POSITIVE_INFINITY;
        }
        distance[source] = 0;
        for (int i = 0; i < numberVertices - 1; i++) {
            for (Vertex u : adjList) {
                for (Vertex v : u.neighbors.keySet()) {
                    if (distance[u.index] + weight(u, v) < distance[v.index]) {
                        distance[v.index] = distance[u.index] + weight(u, v);
                    }
                }
            }
        }
        for (Vertex u : adjList) {
            for (Vertex v : u.neighbors.keySet()) {
                if (distance[u.index] + weight(u, v) < distance[v.index]) {
                    throw new NegativeCycleException("Negative cycle detected");
                }
            }
        }
        return distance;
    }


    public static void main(String[] args) throws NegativeCycleException {
        Scanner sc = new Scanner(System.in);
        int n = sc.nextInt();
        DirectedGraph g = new DirectedGraph(n);
        int m = sc.nextInt();
        for (int i = 0; i < m; i++) {
            g.addEdge(sc.nextInt(), sc.nextInt(), sc.nextInt());
        }
        System.out.println(Arrays.toString(g.bellmanFord(sc.nextInt())));
    }
}