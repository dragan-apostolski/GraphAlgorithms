package Graphs.DirectedWeighted;

import java.util.*;
import java.util.stream.Collectors;

/**
 * A class that represents a directed graph with weighted edges.
 * Every vertex is represented via the {@link Vertex} class, while every edge is
 * internally represented in the same Vertex class.
 *
 * @author drapostolski
 */
public class DirectedGraph implements Cloneable{

    protected int numberVertices;
    protected Vertex[] adjList;


    /**
     * Creates an empty Directed Graph with the given number of vertices. No edges are created.
     *
     * @param numberOfVertices the initial number of vertices
     */
    public DirectedGraph(int numberOfVertices) {
        this.numberVertices = numberOfVertices;
        adjList = new Vertex[numberOfVertices];
        for(int i = 0; i < numberOfVertices; i++)
            adjList[i] = new Vertex(i);
    }

    /**
     * Adds a directed edge (u, v) with the given weight to the graph.
     *
     * @param u the index of the vertex where the edge comes out from
     * @param v the index of the vertex where the edge is directed to
     * @param weight the weight of the directed edge (u, v)
     */
    public void addEdge(int u, int v, double weight){
        adjList[u].addNeighbor(adjList[v], weight);
    }

    /**
     * Removes the directed edge (u,v) from the graph.
     *
     * @param u the index of the source vertex
     * @param v the index of the end vertex
     */
    public void removeEdge(int u, int v){
        adjList[u].removeNeighbor(adjList[v]);
    }

    /**
     * Adds a vertex to this graph.
     *
     * @param v the new vertex to be added
     */
    public void addVertex(Vertex v){
        adjList = Arrays.copyOf(adjList, adjList.length + 1);
        adjList[adjList.length - 1] = v;
        numberVertices = adjList.length;
    }

    public Vertex[] getAdjList() {
        return adjList;
    }

    /**
     * Returns the weight of the directed edge between u and v.
     * @param u the index of the source vertex
     * @param v the index of the end vertex
     * @return the weight of the edge (u, v).
     */
    protected Double weight(Vertex u, Vertex v){
        return adjList[u.index].getWeight(v);
    }

    protected boolean relax(double distance [], Vertex u, Vertex v){
        if (distance[u.index] + weight(u, v) < distance[v.index]) {
            distance[v.index] = distance[u.index] + weight(u, v);
            return true;
        }
        else return false;
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

    /**
     * An implementation of Dijkstra's algorithm with a binary min {@link PriorityQueue}
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
                if(relax(distance, u, v)){
                    v.setDistance(distance[v.index]);
                    pq.add(v);
                    predecessor[v.index] = u.index;
                }
            }
        }
        return distance;
    }

    /**
     * An implementation of the Bellman - Ford algorithm for shortest paths in a graph with negative edge weights.
     * This algorithm computes the shortest paths from the source vertex and checks if a negative weight cycle is
     * reachable from the source vertex. If so, the method throws a {@link NegativeWeightCycleException}, indicating
     * that shortest paths can't be computed from this source vertex. If the graph is one strongly connected component
     * and has a negative weight cycle, a call of this method from any source vertex will always result in a
     * {@link NegativeWeightCycleException}.
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

    /**
     * An implementation of Johnson's algorithm for constructing the all - pair shortest paths matrix in a sparse graph.
     * This method depends on the Bellman-Ford implementation to detect negative weight cycles, after the graph
     * structure is augmented with adding a new source vertex. If such cycle is detected,
     * the {@link NegativeWeightCycleException} is rethrown from the bellmanFord method. It also depends on the
     * Dijkstra's algorithm implementation, now being assured that there are no negative weight cycles, to run it from
     * each source vertex in the process of computing the all - pair shortest paths.
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

    /**
     * An implementation of the Breadth-First Search (BFS) algorithm for computing the level distance from the source
     * (root) vertex to every other reachable vertex in the graph. If a vertex is not reachable from the source vertex,
     * its distance will be marked as positive infinity.
     *
     * The running time of this method is O(V + E).
     *
     * @param source the index of the source vertex
     * @return the array of distances from the source vertex
     */
    public Double [] bfs(int source){
        Double [] distance = new Double[numberVertices];
        Set<Vertex> visited = new HashSet<>();
        for (int i = 0; i < distance.length; i++) {
            distance[i] = Double.POSITIVE_INFINITY;
        }
        distance[source] = 0d;
        Queue<Vertex> q = new LinkedList<>();
        q.add(adjList[source]);
        while(!q.isEmpty()){
            Vertex u = q.remove();
            visited.add(u);
            for (Vertex v : u.neighbors.keySet()) {
                if(!visited.contains(v)){
                    distance[v.index] = distance[u.index] + 1;
                    q.add(v);
                }
            }
        }
        return distance;
    }

    /**
     * A function that checks whether this graph is bipartite, returning true if so, false otherwise.
     *
     * @return a boolean value indicating whether this graph hold the bipartite property
     */
    public boolean isBipartite(){
        int [] partitions = new int[numberVertices];
        Set<Vertex> visited = new HashSet<>();
        partitions[adjList[0].index] = 1;
        visited.add(adjList[0]);
        Queue<Vertex> q = new LinkedList<>();
        q.add(adjList[0]);
        while(!q.isEmpty()){
            Vertex u = q.poll();
            for (Vertex v : u.neighbors.keySet()) {
                if(partitions[u.index] == partitions[v.index]) {
                    return false;
                }
                else if (!visited.contains(v)){
                    partitions[v.index] = 3 - partitions[u.index];
                    q.add(v);
                }
            }
            visited.add(u);
        }
        return true;
    }

    /**
     * This method performs a topological sort of the graph, based on the the finish time of each vertex during the dfs
     * visit method calls.
     * The method returns a {@link TreeSet} of vertices which is sorted in decreasing order of the time they are
     * finished, that is the time they are lastly visited by the dfs visit method.
     *
     * @return topologically sorted set of the vertices in this graph
     */
    public TreeSet<Vertex> topologicalSort(){
        dfs();
        Comparator<Vertex> vertexComparator = Comparator.comparing(Vertex::getTimeFinished).reversed();
        return Arrays.stream(adjList)
                .collect(Collectors.toCollection( () -> new TreeSet<>(vertexComparator)));
    }

    /**
     * This method finds all strongly connected components in the graphs and returns them as sets of vertices in one
     * list, where each set of vertices represents the vertices in one strongly connected component.
     *
     * @return list of all the sets of strongly connected components
     */
    public List<Set<Vertex>> stronglyConnectedComponents(){
        dfs();
        transposeGraph();
        return dfsTransposedEdges();
    }

    private List<Set<Vertex>> findAllDepthFirstTrees(){
        List<Vertex> vertices = Arrays.stream(adjList)
                .sorted(Comparator.comparing(Vertex::getTimeFinished).reversed())
                .collect(Collectors.toList());
        List<Set<Vertex>> stronglyConnectedComponents = new ArrayList<>();
        int i = 0;
        while(i < vertices.size()) {
            Vertex root = vertices.get(i);
            HashSet<Vertex> component = new HashSet<>();
            component.add(root);
            int j;
            for (j = i + 1; j < vertices.size(); j++) {
                Vertex v = vertices.get(j);
                if(v.timeDiscovered > root.timeDiscovered && v.timeFinished < root.timeFinished)
                    component.add(v);
                else break;
            }
            stronglyConnectedComponents.add(component);
            i = j;
        }
        return stronglyConnectedComponents;
    }

    private void transposeGraph(){
        DirectedGraph gTransposed = new DirectedGraph(numberVertices);
        for (int i = 0; i < numberVertices; i++) {
            Vertex u = adjList[i];
            for (Vertex v : u.neighbors.keySet()) {
                gTransposed.addEdge(v.index, u.index, weight(u, v));
            }
            gTransposed.adjList[i].timeFinished = u.timeFinished;
            gTransposed.adjList[i].timeDiscovered = u.timeDiscovered;
        }
        adjList = gTransposed.adjList;
    }


    private Integer time;

    private List<Set<Vertex>> dfsTransposedEdges() {
        time = 0;
        Set<Vertex> visited = new HashSet<>();
        List<Vertex> topologicallySorted = Arrays.stream(adjList)
                .sorted(Comparator.comparing(Vertex::getTimeFinished).reversed())
                .collect(Collectors.toList());
        List<Set<Vertex>> stronglyConnectedComponents = new ArrayList<>();
        for (Vertex u : topologicallySorted) {
            if(!visited.contains(u)) {
                HashSet<Vertex> component = new HashSet<>();
                dfsVisit(visited, component, u);
                stronglyConnectedComponents.add(component);
            }
        }
        return stronglyConnectedComponents;
    }

    private void dfsVisit(Set<Vertex> visited, HashSet<Vertex> component,  Vertex u) {
        u.timeDiscovered = ++time;
        component.add(u);
        visited.add(u);
        for (Vertex v : u.neighbors.keySet()) {
            if(!visited.contains(v)){
                dfsVisit(visited, component, v);
            }
        }
        u.timeFinished = ++time;
    }

    private Integer[] dfs(){
        time = 0;
        Integer [] predecessor = new Integer[numberVertices];
        Set<Vertex> visited = new HashSet<>();
        for (Vertex u : adjList) {
            if(!visited.contains(u))
                dfsVisit(visited, predecessor, u);
        }
        return predecessor;
    }

    private void dfsVisit(Set<Vertex> visited, Integer[] predecessor, Vertex u) {
        u.timeDiscovered = ++time;
        visited.add(u);
        for (Vertex v : u.neighbors.keySet()) {
            if(!visited.contains(v)){
                predecessor[v.index] = u.index;
                dfsVisit(visited, predecessor, v);
            }
        }
        u.timeFinished = ++time;
    }
}